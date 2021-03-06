from rlbot.utils.structures.bot_input_struct import PlayerInput
from rlbot.utils.structures.game_data_struct import Rotator, Vector3, PlayerInfo, GameTickPacket

from hover import Hover
from rlutilities.linear_algebra import vec3, mat3, euler_to_rotation, angle_between, xy, dot
from rlutilities.mechanics import Aerial, Reorient, Drive
from rlutilities.simulation import Car, Input


class Drone(Car):

    def __init__(self, index: int, team: int, airshow_id: int):
        super().__init__()
        self.team = team
        self.id = index
        self.airshow_id = airshow_id
        self.reorient = Reorient(self)
        self.aerial = Aerial(self)
        self.hover = Hover(self)
        self.drive = Drive(self)

    def update(self, game_car: PlayerInfo, packet: GameTickPacket):
        self.position = vector3_to_vec3(game_car.physics.location)
        self.velocity = vector3_to_vec3(game_car.physics.velocity)
        self.orientation = rotator_to_mat3(game_car.physics.rotation)
        self.angular_velocity = vector3_to_vec3(game_car.physics.angular_velocity)
        self.boost = game_car.boost
        self.time = packet.game_info.seconds_elapsed
        self.on_ground = game_car.has_wheel_contact
        self.jumped = game_car.jumped
        self.double_jumped = game_car.double_jumped

        self.reorient = Reorient(self)
        self.controls = Input()

    def get_player_input(self) -> PlayerInput:
        player_input = PlayerInput()
        player_input.throttle = self.controls.throttle
        player_input.steer = self.controls.steer
        player_input.pitch = self.controls.pitch
        player_input.yaw = self.controls.yaw
        player_input.roll = self.controls.roll
        player_input.jump = self.controls.jump
        player_input.boost = self.controls.boost
        player_input.handbrake = self.controls.handbrake
        return player_input


def vector3_to_vec3(v: Vector3) -> vec3:
    return vec3(v.x, v.y, v.z)


def rotator_to_mat3(r: Rotator) -> mat3:
    return euler_to_rotation(vec3(r.pitch, r.yaw, r.roll))


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


def aerial(drone: Drone, target: vec3, delta_time: float):
    drone.aerial.target_position = target
    drone.aerial.arrival_time = drone.time + delta_time
    drone.aerial.step(1 / 120)
    drone.controls = drone.aerial.controls


def hover(drone: Drone, target: vec3):
    drone.hover.target = target
    drone.hover.step(1 / 120)
    drone.controls = drone.hover.controls
