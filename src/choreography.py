from dataclasses import dataclass
from math import pi
from typing import Dict, Optional, Tuple, List, Callable

from rlbot.utils.game_state_util import GameState, BallState, Physics, Vector3, CarState
from rlbot.utils.structures.bot_input_struct import PlayerInput
from rlbot.utils.structures.game_data_struct import GameTickPacket, PlayerInfo
from rlbot.utils.structures.game_interface import GameInterface

from drone import Drone
from rlutilities.linear_algebra import vec3, mat3, look_at, axis_to_rotation, dot, inv, cross, normalize, norm, vec2, \
    sgn, angle_between, xy
from rlutilities.simulation import Game
from steps import Step, StepResult, CompositeStep, Wait, ParallelStep, PartialStep, StepContext, \
    make_physics

inner_group_bot_count = 20
choreo_human_index = 40


def find_human(packet: GameTickPacket) -> Optional[Tuple[int, PlayerInfo]]:
    human_players = [(i, car) for i, car in enumerate(packet.game_cars[:packet.num_cars]) if not car.is_bot]
    return human_players[0] if human_players else None


Game.set_mode("soccar")


class Choreography:
    def __init__(self, interface: GameInterface, packet: GameTickPacket):
        self.interface = interface
        index_offset = -1 if find_human(packet) is not None else 0
        self.drones = [Drone(i, packet.game_cars[i].team,
                             (i if i + index_offset < choreo_human_index else i + 1) + index_offset)
                       for i in range(packet.num_cars) if packet.game_cars[i].is_bot]

        self.step: Optional[Step] = None
        self.last_reset_time = 0

    def hide_ball(self):
        self.interface.set_game_state(GameState(ball=BallState(Physics(
            location=Vector3(0, 0, -100),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0),
        ))))

    def get_outputs(self, packet: GameTickPacket) -> Dict[int, PlayerInput]:
        if self.step is None:
            self.generate_sequence()
            self.last_reset_time = packet.game_info.seconds_elapsed
            self.hide_ball()

        for drone in self.drones:
            drone.update(packet.game_cars[drone.id], packet)

        player = find_human(packet)
        if player:
            player_index, player_info = player
            player = Drone(player_index, player_info.team, choreo_human_index)
            player.update(player_info, packet)

        t = packet.game_info.seconds_elapsed - self.last_reset_time
        result = self.step.perform(StepContext(self.drones, self.interface, player), t)

        if result.car_states:
            self.interface.set_game_state(GameState(cars=result.car_states or None))

        if result.finished:
            self.step = None

        return {drone.id: drone.get_player_input() for drone in self.drones}

    def generate_sequence(self):
        def airshow(includes_player):
            repair = lambda: CompositeStep(steps=[
                RepairFormation(include_player=includes_player),
                Wait(0.5),
            ])
            return [
                Wait(1.0),

                Dodge(direction=vec2(1, 0)),
                Wait(2.0),
                repair(),

                Dodge(direction=vec2(-1, 0)),
                Wait(2.0),
                repair(),

                Dodge(direction=vec2(0, -1)),
                Wait(2.0),
                repair(),

                Dodge(direction=vec2(0, 1)),
                Wait(2.0),
                repair(),

                JumpAndTurn(direction=vec2(-1, 0)),
                repair(),

                PolarDrive(duration=1.0, speed=1000, angular_direction=0, boost=True, radius_offset=3000),
                GroundStop(),
                repair(),

                JumpAndTurn(direction=vec2(1, 0)),
                repair(),

                PolarDrive(duration=1.0, speed=1000, angular_direction=0, boost=True),
                GroundStop(),
                repair(),

                JumpAndTurn(direction=vec2(0, 1)),
                repair(),

                PolarDrive(duration=5.0, speed=1000),
                GroundStop(),
                repair(),

                JumpAndTurn(direction=vec2(0, -1)),
                repair(),

                PolarDrive(duration=5.0, speed=1000, angular_direction=-1),
                GroundStop(),
                repair(),

                Wait(1.0),
            ]

        self.step = CompositeStep(steps=[
            SetCircle(),
            ParallelStep(steps=[
                PartialStep(
                    airshow_ids=range(inner_group_bot_count),
                    step=CompositeStep(steps=airshow(includes_player=False))
                ),
                PartialStep(
                    airshow_ids=range(inner_group_bot_count, 64),
                    step=EvaluateStep(steps=[Wait(0.5)] + airshow(includes_player=True))
                ),
            ]),
        ])


def rotation(drone_id: int, angular_offset=0.0) -> mat3:
    if drone_id < inner_group_bot_count:
        angle = drone_id / inner_group_bot_count * pi * 2
        angle += angular_offset * 2
    else:
        angle = (drone_id - inner_group_bot_count) / (64 - inner_group_bot_count) * pi * 2
        angle += angular_offset
    return axis_to_rotation(vec3(0, 0, angle))


def circle_radius(drone_id: int) -> float:
    return 1200 if drone_id < inner_group_bot_count else 2000


def circle_pos(drone_id: int, radius_offset=0.0, angular_offset=0.0) -> vec3:
    rot = rotation(drone_id, angular_offset)
    return dot(rot, vec3(1, 0, 0)) * (circle_radius(drone_id) + radius_offset)


def polar_mean(drones: List[Drone], value: Callable[[Drone], vec3]) -> vec3:
    sum = vec3()
    for drone in drones:
        sum += dot(inv(rotation(drone.airshow_id)), value(drone))
    return sum / len(drones)


class EvaluateStep(CompositeStep):
    def perform(self, context: StepContext, t: float) -> StepResult:
        result = super().perform(context, t)

        human_rot = rotation(context.player.airshow_id)
        correct_pos = dot(human_rot, polar_mean(context.drones, lambda drone: drone.position))
        correct_forward = dot(human_rot, polar_mean(context.drones, lambda drone: drone.forward()))
        correct_up = dot(human_rot, polar_mean(context.drones, lambda drone: drone.up()))
        correct_left = normalize(cross(correct_up, correct_forward))

        renderer = context.interface.renderer
        renderer.begin_rendering()
        length, width = 120, 85
        corners = [
            correct_pos + correct_left * width / 2 + correct_forward * length / 2,
            correct_pos - correct_left * width / 2 + correct_forward * length / 2,
            correct_pos - correct_left * width / 2 - correct_forward * length / 2,
            correct_pos + correct_left * width / 2 - correct_forward * length / 2,
        ]
        # renderer.draw_polyline_3d(corners + [corners[0]], renderer.yellow())
        renderer.end_rendering()
        return result


@dataclass
class RepairFormation(Step):
    duration: float = 0.5
    include_player: bool = False

    def perform(self, context: StepContext, t: float) -> StepResult:
        drones_with_player = context.drones + ([context.player] if context.player and self.include_player else [])

        mean_pos = polar_mean(context.drones, lambda drone: drone.position)
        mean_vel = polar_mean(context.drones, lambda drone: drone.velocity)
        mean_angvel = polar_mean(context.drones, lambda drone: drone.angular_velocity)
        mean_forward = polar_mean(context.drones, lambda drone: drone.forward())
        mean_up = polar_mean(context.drones, lambda drone: drone.up())

        car_states = {}
        for drone in drones_with_player:
            rot = rotation(drone.airshow_id)
            pos = dot(rot, mean_pos)
            vel = dot(rot, mean_vel)
            angvel = dot(rot, mean_angvel)
            forward = dot(rot, mean_forward)
            up = dot(rot, mean_up)
            if (
                    norm(pos - drone.position) > 200
                    or norm(vel - drone.velocity) > 500
                    or norm(forward - drone.forward()) > 0.5
                    or norm(up - drone.up()) > 0.5
            ):
                car_states[drone.id] = CarState(physics=make_physics(pos, look_at(forward, up), vel, angvel))

        return StepResult(finished=t > self.duration, car_states=car_states)


def reorient(drone: Drone, target: mat3):
    drone.reorient.target_orientation = target
    drone.reorient.step(1 / 120)
    drone.controls = drone.reorient.controls
    if angle_between(xy(drone.forward()), xy(dot(target, vec3(1, 0, 0)))) > 3.0:
        drone.controls.yaw = 1.0


def drive(drone: Drone, target: vec3, speed: float):
    drone.drive.target = target
    drone.drive.speed = speed
    drone.drive.step(1 / 120)
    drone.controls = drone.drive.controls


@dataclass
class SetCircle(Step):
    duration: float = 0.5

    def perform(self, context: StepContext, t: float) -> StepResult:
        car_states = {drone.id: CarState(physics=make_physics(
            pos=circle_pos(drone.airshow_id, angular_offset=1.0),
            ori=look_at(
                dot(rotation(drone.airshow_id, angular_offset=1.0), vec3(-1, 0, 0))
            ),
        )) for drone in context.drones + [context.player]}

        for state in car_states.values():
            state.physics.location.z = 17

        result = self.result(t)
        result.car_states = car_states
        return result


def direction_on_circle(drone_pos: vec3, direction: vec2) -> vec3:
    towards_center = normalize(drone_pos) * -1
    tangent = normalize(cross(towards_center, vec3(0, 0, 1)))
    return direction.x * towards_center + direction.y * tangent


def position_on_circle(drone_pos: vec3, radius: float, angle_offset: float) -> vec3:
    return dot(axis_to_rotation(vec3(0, 0, angle_offset)), normalize(drone_pos)) * radius


@dataclass
class JumpAndTurn(Step):
    duration: float = 2.3
    direction: vec2 = vec2(1, 0)

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            if t < 0.2:
                drone.controls.jump = True
            else:
                reorient(drone, look_at(direction_on_circle(drone.position, self.direction)))

        return self.result(t)


@dataclass
class Dodge(Step):
    duration: float = 0.4
    direction: vec2 = vec2(1, 0)

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            if t < 0.2:
                drone.controls.jump = True
            elif t > 0.3:
                drone.controls.jump = True
                drone.controls.pitch = -self.direction.x
                drone.controls.yaw = self.direction.y

        return self.result(t)


@dataclass
class JumpAndFlyUp(Step):
    duration: float = 3.0

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            reorient(drone, look_at(vec3(0, 0, 1), drone.position))
            if t < 0.4:
                drone.controls.jump = True
            else:
                drone.controls.boost = True
        return self.result(t)


@dataclass
class PolarDrive(Step):
    radius_offset: float = 0.0
    speed: float = 1400
    angular_direction: int = 1
    boost: bool = False

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            drive(drone, position_on_circle(
                drone_pos=drone.position,
                radius=circle_radius(drone.airshow_id) + self.radius_offset,
                angle_offset=0.2 * self.angular_direction
            ), self.speed)

            if not self.boost:
                drone.controls.boost = False

        return self.result(t)


@dataclass
class GroundStop(Step):
    duration: float = 0.7

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            vf = dot(drone.forward(), drone.velocity)
            if abs(vf) > 100:
                drone.controls.throttle = -sgn(vf)
        return self.result(t)
