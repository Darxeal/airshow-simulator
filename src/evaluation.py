from dataclasses import dataclass

from rlbot.utils.game_state_util import CarState

from drone import Drone
from polar_utils import rotation, polar_mean
from rlutilities.linear_algebra import vec3, norm, angle_between, dot, normalize, cross, look_at
from steps import CompositeStep, StepContext, StepResult, Step, make_physics


def drone_error(drone: Drone, pos: vec3, vel: vec3, forward: vec3, up: vec3) -> float:
    error = 0
    error += norm(pos - drone.position) * 0.03
    error += norm(vel - drone.velocity) * 0.001
    error += max(angle_between(forward, drone.forward()), angle_between(up, drone.up())) * 2.0
    return error


@dataclass
class EvaluateStep(CompositeStep):
    render_rectangle: bool = False
    render_eval_text: bool = True
    score = 0
    frame_counter = 0

    def perform(self, context: StepContext, t: float) -> StepResult:
        result = super().perform(context, t)

        human_rot = rotation(context.player.airshow_id)
        correct_pos = dot(human_rot, polar_mean(context.drones, lambda drone: drone.position))
        correct_vel = dot(human_rot, polar_mean(context.drones, lambda drone: drone.velocity))
        correct_forward = dot(human_rot, polar_mean(context.drones, lambda drone: drone.forward()))
        correct_up = dot(human_rot, polar_mean(context.drones, lambda drone: drone.up()))
        correct_left = normalize(cross(correct_up, correct_forward))

        error = drone_error(context.player, correct_pos, correct_vel, correct_forward, correct_up)
        EvaluateStep.score += (1 - error / 10)
        EvaluateStep.frame_counter += 1

        renderer = context.interface.renderer
        renderer.begin_rendering()

        if self.render_rectangle:
            length, width = 120, 85
            corners = [
                correct_pos + correct_left * width / 2 + correct_forward * length / 2,
                correct_pos - correct_left * width / 2 + correct_forward * length / 2,
                correct_pos - correct_left * width / 2 - correct_forward * length / 2,
                correct_pos + correct_left * width / 2 - correct_forward * length / 2,
            ]
            renderer.draw_polyline_3d(corners + [corners[0]], renderer.yellow())

        if self.render_eval_text:
            if error < 0.5:
                text, color = "Perfect", renderer.lime()
            elif error < 5.0:
                text, color = "Okay", renderer.lime()
            elif error < 10.0:
                text, color = "Bad", renderer.yellow()
            else:
                text, color = "?????", renderer.red()

            renderer.draw_string_3d(context.player.position, 2, 2, text, color)

        renderer.end_rendering()
        return result

    @classmethod
    def reset_score(cls):
        cls.score = 0
        cls.frame_counter = 0

    @classmethod
    def get_score(cls) -> int:
        return int(cls.score / max(cls.frame_counter, 1) * 1000)


@dataclass
class DisplayText(Step):
    text: str = ""

    def perform(self, context: StepContext, t: float) -> StepResult:
        renderer = context.interface.renderer
        renderer.draw_string_3d(context.player.position, 3, 3, self.text, renderer.yellow())
        return self.result(t)


@dataclass
class DisplayScore(Step):
    text: str = ""

    def perform(self, context: StepContext, t: float) -> StepResult:
        renderer = context.interface.renderer
        renderer.draw_string_3d(context.player.position, 5, 5, f"Score: {EvaluateStep.get_score()}", renderer.yellow())
        return self.result(t)


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
            error = drone_error(drone, pos, vel, forward, up)
            if error > (5.0 if drone is context.player else 1.0):
                car_states[drone.id] = CarState(physics=make_physics(pos, look_at(forward, up), vel, angvel))

        return StepResult(finished=t > self.duration, car_states=car_states)
