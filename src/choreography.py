from dataclasses import dataclass
from math import pi

from rlbot.utils.game_state_util import BallState, Physics, Vector3, CarState

from drone import reorient, drive, hover
from evaluation import EvaluateStep, RepairFormation, DisplayScore, DisplayText
from polar_utils import rotation, circle_radius, circle_pos, direction_on_circle, position_on_circle, \
    inner_group_bot_count
from rlutilities.linear_algebra import vec3, look_at, dot, vec2, \
    sgn
from rlutilities.simulation import Game
from step_runner import StepRunner
from steps import Step, StepResult, CompositeStep, Wait, ParallelStep, PartialStep, StepContext, \
    make_physics, vec3_to_vector3

Game.set_mode("soccar")


class Choreography(StepRunner):
    choreo_human_index = 40

    def generate_sequence(self):
        def airshow(includes_player):
            repair = lambda: CompositeStep(steps=[
                RepairFormation(include_player=includes_player),
                Wait(0.5),
            ])

            return [

                # JUMP TWICE
                JumpAndTurn(direction=vec2(1, 0)),
                repair(),
                JumpAndTurn(direction=vec2(1, 0)),
                repair(),

                # JUMP TURN BACK AND FORTH
                JumpAndTurn(direction=vec2(-1, 0)),
                repair(),
                JumpAndTurn(direction=vec2(1, 0)),
                repair(),

                # DODGE FORWARD AND BACK
                JumpAndDodge(direction=vec2(1, 0)),
                Wait(2.0),
                repair(),
                JumpAndDodge(direction=vec2(-1, 0)),
                Wait(2.0),
                repair(),

                # DODGE FORWARD AND BACK
                JumpAndDodge(direction=vec2(1, 0)),
                Wait(2.0),
                repair(),
                JumpAndDodge(direction=vec2(-1, 0)),
                Wait(2.0),
                repair(),

                # DODGE LEFT AND RIGHT
                JumpAndDodge(direction=vec2(0, -1)),
                Wait(2.0),
                repair(),
                JumpAndDodge(direction=vec2(0, 1)),
                Wait(2.0),
                repair(),

                # DRIVE OUT AND BACK
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

                # DRIVE CLOCKWISE AND COUNTER-CLOCKWISE
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
                JumpAndTurn(direction=vec2(1, 0)),
                repair(),

                # BOOST UP, FALL DOWN
                JumpAndFlyUp(1.5),
                Wait(1.0),
                LandSmoothly(duration=1.0, direction=vec2(1, 0)),
                repair(),

                # BOOST UP, FALL DOWN (HIGHER)
                JumpAndFlyUp(2.0),
                Wait(1.0),
                LandSmoothly(duration=2.0, direction=vec2(1, 0)),
                repair(),

                # BOOST UP AND THEN DOWN
                JumpAndFlyUp(2.0),
                PolarReorient(0.3, forward=vec3(1, 0, 0), up=vec3(0, 0, 1)),
                PolarReorient(0.5, forward=vec3(0, 0, -1), up=vec3(1, 0, 0)),
                PolarReorient(0.5, forward=vec3(0, 0, -1), up=vec3(1, 0, 0), boost=True),
                LandSmoothly(duration=1.0, direction=vec2(1, 0)),
                repair(),

                # BOOST UP AND THEN DOWN (HIGHER)
                JumpAndFlyUp(2.5),
                PolarReorient(0.3, forward=vec3(1, 0, 0), up=vec3(0, 0, 1)),
                PolarReorient(0.5, forward=vec3(0, 0, -1), up=vec3(1, 0, 0)),
                PolarReorient(0.8, forward=vec3(0, 0, -1), up=vec3(1, 0, 0), boost=True),
                LandSmoothly(duration=2.0, direction=vec2(1, 0)),
                repair(),

                # DRIVE CLOCKWISE AND COUNTER-CLOCKWISE
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
                JumpAndTurn(direction=vec2(1, 0)),
                repair(),

                # FINAL AIRSHOW
                JumpAndFlyUp(1.5),
                PolarFlight(duration=3.0),
                RepairFormation(duration=0.1, include_player=includes_player),
                PolarFlight(duration=20.0),
                RepairFormation(duration=0.1, include_player=includes_player),
                PolarFlight(angular_speed=0.4, duration=pi * 2 / 0.4),
                PolarFlight(duration=5.0),
                RepairFormation(duration=0.1, include_player=includes_player),
                PolarFlight(angular_speed=-0.4, duration=pi * 2 / 0.4),
                PolarFlight(duration=10.0),
                RepairFormation(duration=0.1, include_player=includes_player),
                Wait(0.5),
                LandSmoothly(duration=3.0),
            ]

        self.step = CompositeStep(steps=[
            SetCircle(),
            TeleportBall(pos=vec3(0, 0, -100)),
            Wait(1.0),
            ParallelStep(steps=[
                DisplayText(text="3"),
                PartialStep(step=RepairFormation(1.0, True), airshow_ids=range(inner_group_bot_count, 64)),
            ]),
            ParallelStep(steps=[
                DisplayText(text="2"),
                PartialStep(step=RepairFormation(1.0, True), airshow_ids=range(inner_group_bot_count, 64)),
            ]),
            ParallelStep(steps=[
                DisplayText(text="1"),
                PartialStep(step=RepairFormation(1.0, True), airshow_ids=range(inner_group_bot_count, 64)),
            ]),
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
            DisplayScore(duration=10.0),
            DisplayText(duration=1.0, text="Next round in 3"),
            DisplayText(duration=1.0, text="Next round in 2"),
            DisplayText(duration=1.0, text="Next round in 1"),
        ])


class LoadGoalExplosion(Step):
    def perform(self, context: StepContext, t: float) -> StepResult:
        return StepResult(finished=t > 0.1, ball_state=BallState(Physics(
            location=Vector3(0, 0, 100),
        )), car_states={0: CarState(Physics(
            location=Vector3(0, 0, 17),
        ))})


@dataclass
class TeleportBall(Step):
    pos: vec3 = vec3(0, 0, 93)

    def perform(self, context: StepContext, t: float) -> StepResult:
        return StepResult(finished=True, ball_state=BallState(Physics(
            location=vec3_to_vector3(self.pos),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0),
        )))


@dataclass
class SetCircle(Step):
    duration: float = 0.5

    def perform(self, context: StepContext, t: float) -> StepResult:
        car_states = {drone.id: CarState(physics=make_physics(
            pos=circle_pos(drone.airshow_id),
            ori=look_at(
                dot(rotation(drone.airshow_id), vec3(-1, 0, 0))
            ),
        )) for drone in context.drones + [context.player]}

        for state in car_states.values():
            state.physics.location.z = 17

        result = self.result(t)
        result.car_states = car_states
        return result


@dataclass
class JumpAndTurn(Step):
    duration: float = 2.3
    direction: vec2 = vec2(1, 0)

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            if t < 0.2:
                drone.controls.jump = True
            else:
                reorient(drone, look_at(direction_on_circle(drone.position, vec3(self.direction))))

        return self.result(t)


@dataclass
class LandSmoothly(Step):
    direction: vec2 = vec2(1, 0)
    up_z: float = 1

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            reorient(drone, look_at(direction_on_circle(drone.position, vec3(self.direction)), vec3(0, 0, self.up_z)))
        return self.result(t)


@dataclass
class PolarReorient(Step):
    forward: vec3 = vec3(1, 0, 0)
    up: vec3 = vec3(0, 0, 1)
    boost: bool = False

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            reorient(drone, look_at(
                direction_on_circle(drone.position, vec3(self.forward)),
                direction_on_circle(drone.position, vec3(self.up))
            ))
            drone.controls.boost = self.boost
        return self.result(t)


@dataclass
class JumpAndDodge(Step):
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
class InstantDodge(Step):
    duration: float = 0.4
    direction: vec2 = vec2(1, 0)

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            drone.controls.jump = True
            drone.controls.pitch = -self.direction.x
            drone.controls.yaw = self.direction.y

        return self.result(t)


@dataclass
class JumpAndFlyUp(Step):
    duration: float = 3.0

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            if t > 0.2:
                reorient(drone, look_at(vec3(0, 0, 1), drone.position))
            if t < 0.8:
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


@dataclass
class PolarFlight(Step):
    height: float = 700
    angular_speed: float = 0.0

    def perform(self, context: StepContext, t: float) -> StepResult:
        for drone in context.drones:
            target = circle_pos(drone.airshow_id, angular_offset=t * self.angular_speed)
            target.z = 1000
            drone.hover.up = drone.position
            hover(drone, target=target)
        return self.result(t)
