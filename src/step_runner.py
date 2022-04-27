from typing import Optional, Dict, Tuple

from rlbot.utils.game_state_util import GameState
from rlbot.utils.structures.bot_input_struct import PlayerInput
from rlbot.utils.structures.game_data_struct import GameTickPacket, PlayerInfo
from rlbot.utils.structures.game_interface import GameInterface

from drone import Drone
from evaluation import EvaluateStep
from steps import Step, StepContext


class StepRunner:
    choreo_human_index = None

    def __init__(self, interface: GameInterface, packet: GameTickPacket):
        self.interface = interface
        index_offset = -1 if find_human(packet) is not None else 0
        self.drones = [Drone(i, packet.game_cars[i].team,
                             (i if i + index_offset < self.choreo_human_index else i + 1) + index_offset)
                       for i in range(packet.num_cars) if packet.game_cars[i].is_bot]

        self.step: Optional[Step] = None
        self.last_reset_time = 0

    def get_outputs(self, packet: GameTickPacket) -> Dict[int, PlayerInput]:
        if self.step is None:
            self.generate_sequence()
            EvaluateStep.reset_score()
            self.last_reset_time = packet.game_info.seconds_elapsed

        for drone in self.drones:
            drone.update(packet.game_cars[drone.id], packet)

        player = find_human(packet)
        if player:
            player_index, player_info = player
            player = Drone(player_index, player_info.team, self.choreo_human_index)
            player.update(player_info, packet)

        t = packet.game_info.seconds_elapsed - self.last_reset_time
        self.interface.renderer.begin_rendering()
        result = self.step.perform(StepContext(self.drones, self.interface, player), t)
        self.interface.renderer.end_rendering()

        if result.car_states or result.ball_state:
            self.interface.set_game_state(GameState(cars=result.car_states or None, ball=result.ball_state))

        if result.finished:
            self.step = None

        return {drone.id: drone.get_player_input() for drone in self.drones}

    def generate_sequence(self):
        raise NotImplementedError


def find_human(packet: GameTickPacket) -> Optional[Tuple[int, PlayerInfo]]:
    human_players = [(i, car) for i, car in enumerate(packet.game_cars[:packet.num_cars]) if not car.is_bot]
    return human_players[0] if human_players else None
