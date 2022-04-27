from math import pi
from typing import List, Callable

from drone import Drone
from rlutilities.linear_algebra import mat3, axis_to_rotation, vec3, dot, inv, normalize, cross


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


def direction_on_circle(drone_pos: vec3, direction: vec3) -> vec3:
    towards_center = normalize(drone_pos) * -1
    tangent = normalize(cross(towards_center, vec3(0, 0, 1)))
    return direction.x * towards_center + direction.y * tangent + direction.z * vec3(0, 0, 1)


def position_on_circle(drone_pos: vec3, radius: float, angle_offset: float) -> vec3:
    return dot(axis_to_rotation(vec3(0, 0, angle_offset)), normalize(drone_pos)) * radius


inner_group_bot_count = 20
