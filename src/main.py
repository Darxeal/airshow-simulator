import importlib
import time
import traceback
from pathlib import Path
from types import ModuleType

from rlbot.agents.base_script import BaseScript
from rlbot.matchconfig.match_config import PlayerConfig, MatchConfig, MutatorConfig
from rlbot.setup_manager import SetupManager

import choreography


def human_config():
    player_config = PlayerConfig()
    player_config.bot = False
    player_config.team = 0
    player_config.name = "Human"
    return player_config


def create_player_config(name: str):
    player_config = PlayerConfig()
    player_config.bot = True
    player_config.rlbot_controlled = True
    player_config.name = name
    player_config.team = 0
    return player_config


def build_match_config():
    match_config = MatchConfig()
    match_config.player_configs = [create_player_config(str(i)) for i in range(63)] + [human_config()]
    match_config.game_mode = 'Soccer'
    match_config.game_map = 'Mannfield'
    match_config.existing_match_behavior = 'Continue And Spawn'
    match_config.mutators = MutatorConfig()
    match_config.mutators.boost_amount = "Unlimited"
    match_config.mutators.match_length = "Unlimited"
    match_config.enable_state_setting = True
    match_config.enable_rendering = True
    return match_config


def rreload(module):
    """Recursively reload modules."""
    importlib.reload(module)
    for attribute_name in dir(module):
        attribute = getattr(module, attribute_name)
        if type(attribute) is ModuleType:
            rreload(attribute)


class AirshowSimulator(BaseScript):
    def __init__(self):
        super().__init__("Airshow Simulator")
        self.setup_manager = SetupManager()
        self.setup_manager.game_interface = self.game_interface

        while True:
            packet = self.wait_game_tick_packet()
            if packet.game_info.is_round_active:
                break

        # copied this from TrackAndField, without this rlbot crashes for some reason
        self.setup_manager.num_participants = 0
        self.setup_manager.launch_bot_processes(MatchConfig())

        self.setup_manager.load_match_config(build_match_config())
        self.setup_manager.start_match()

        packet = self.wait_game_tick_packet()
        self.choreo = choreography.Choreography(self.game_interface, packet)

        self.choreo_file = Path(__file__).parent / "choreography.py"
        self.last_mtime = self.choreo_file.lstat().st_mtime

    def run(self):
        while True:
            packet = self.wait_game_tick_packet()

            # reload choreo if modified
            mtime = self.choreo_file.lstat().st_mtime
            if mtime > self.last_mtime:
                try:
                    rreload(choreography)
                    self.choreo = choreography.Choreography(self.game_interface, packet)
                    print(f"[{mtime}] Reloaded choreo")
                    self.last_mtime = mtime
                except Exception as ex:
                    print()
                    print("-----------------RELOAD EXCEPTION-----------------")
                    print(ex)
                    print(traceback.format_exc())

            try:
                controls = self.choreo.get_outputs(packet)
            except Exception as ex:
                print()
                print("-----------------STEP EXCEPTION-----------------")
                print(ex)
                print(traceback.format_exc())
                time.sleep(1.0)
                continue

            for index in controls:
                self.game_interface.update_player_input(controls[index], index)


if __name__ == '__main__':
    script = AirshowSimulator()
    script.run()
