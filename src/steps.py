import dataclasses
from dataclasses import dataclass, field
from typing import Dict, List, Collection, Optional

from rlbot.utils.game_state_util import CarState, Physics, Vector3, Rotator, BallState
from rlbot.utils.structures.game_interface import GameInterface

from drone import Drone
from rlutilities.linear_algebra import vec3, mat3, rotation_to_euler


@dataclass
class StepResult:
    finished: bool = False
    car_states: Dict[int, CarState] = field(default_factory=lambda: {})
    ball_state: Optional[BallState] = None

    def __add__(self, other: "StepResult") -> "StepResult":
        return StepResult(
            finished=self.finished or other.finished,
            car_states={**self.car_states, **other.car_states},
            ball_state=self.ball_state or other.ball_state,
        )


@dataclass
class StepContext:
    drones: List[Drone]
    interface: GameInterface
    player: Optional[Drone]


@dataclass
class Step:
    duration: float = float("inf")

    def result(self, t: float):
        return StepResult(finished=t > self.duration)

    def perform(self, context: StepContext, t: float) -> StepResult:
        raise NotImplementedError


@dataclass
class CompositeStep(Step):
    steps: List[Step] = field(default_factory=lambda: [])

    def __post_init__(self):
        self.current_step_index = 0
        self.current_step_start_t = 0

    def perform(self, context: StepContext, t: float) -> StepResult:
        result = self.steps[self.current_step_index].perform(context, t - self.current_step_start_t)
        if result.finished:
            self.current_step_index += 1
            self.current_step_start_t = t
            if self.current_step_index < len(self.steps):
                result.finished = False
        return result


@dataclass
class ParallelStep(Step):
    steps: List[Step] = field(default_factory=lambda: [])

    def perform(self, context: StepContext, t: float) -> StepResult:
        result = StepResult()
        for step in self.steps:
            result += step.perform(context, t)
        return result


@dataclass
class PartialStep(Step):
    step: Step = None
    airshow_ids: Collection = None

    def perform(self, context: StepContext, t: float) -> StepResult:
        new_context = dataclasses.replace(context)
        new_context.drones = [drone for drone in context.drones if drone.airshow_id in self.airshow_ids]
        return self.step.perform(new_context, t)


def vec3_to_vector3(v: vec3) -> Vector3:
    return Vector3(v.x, v.y, v.z)


def mat3_to_rotator(mat: mat3) -> Rotator:
    pyr = rotation_to_euler(mat)
    return Rotator(pyr.x, pyr.y, pyr.z)


def make_physics(pos: vec3, ori: mat3, vel: vec3 = None, angvel: vec3 = None) -> Physics:
    return Physics(
        location=vec3_to_vector3(pos),
        rotation=mat3_to_rotator(ori),
        velocity=vec3_to_vector3(vel or vec3(0, 0, 0)),
        angular_velocity=vec3_to_vector3(angvel or vec3(0, 0, 0)),
    )


class Wait(Step):
    def perform(self, context: StepContext, t: float) -> StepResult:
        return self.result(t)
