# mypy: ignore-errors
import json
from pathlib import Path


def set_path():

    PATH_TOP = Path(__file__).parent.parent.parent
    PATH_SETTING = PATH_TOP / '3_Setting/Noshiro_sea_2023'

    PATH_INFORMATION = (PATH_SETTING / 'Information.json').as_posix()
    PATH_LAUNCHER = (PATH_SETTING / 'Launcher.json').as_posix()
    PATH_MAP = (PATH_SETTING / 'Map.json').as_posix()
    PATH_PHYSICS = (PATH_SETTING / 'Physics.json').as_posix()
    PATH_ROCKET = (PATH_SETTING / 'Rocket.json').as_posix()
    PATH_SIMULATOR = (PATH_SETTING / 'Simulator.json').as_posix()
    PATH_THRUST = (PATH_SETTING / 'Thrust.json').as_posix()
    PATH_WIND = (PATH_SETTING / 'Wind.json').as_posix()

    return {'PATH_SETTING': PATH_SETTING.as_posix(),
            'PATH_INFORMATION': PATH_INFORMATION,
            'PATH_LAUNCHER': PATH_LAUNCHER,
            'PATH_MAP': PATH_MAP,
            'PATH_PHYSICS': PATH_PHYSICS,
            'PATH_ROCKET': PATH_ROCKET,
            'PATH_SIMULATOR': PATH_SIMULATOR,
            'PATH_THRUST': PATH_THRUST,
            'PATH_WIND': PATH_WIND}


class Constants:
    def __init__(self):

        self.PATH_INFORMATION = set_path()['PATH_INFORMATION']
        self.PATH_LAUNCHER = set_path()['PATH_LAUNCHER']
        self.PATH_MAP = set_path()['PATH_MAP']
        self.PATH_PHYSICS = set_path()['PATH_PHYSICS']
        self.PATH_ROCKET = set_path()['PATH_ROCKET']
        self.PATH_SIMULATOR = set_path()['PATH_SIMULATOR']
        self.PATH_THRUST = set_path()['PATH_THRUST']
        self.PATH_WIND = set_path()['PATH_WIND']

    def information(self):

        with open(self.PATH_INFORMATION, 'r') as Information:
            json_Information = json.load(Information)

        return json_Information['Basic_Information']

    def launcher(self):

        with open(self.PATH_LAUNCHER, 'r') as Launcher:
            json_Launcher = json.load(Launcher)

        return {k: v['Value'] for k, v in json_Launcher.items()}

    def map(self):

        with open(self.PATH_MAP, 'r') as Map:
            json_Map = json.load(Map)

        return {k0: {k1: v1['Value'] for k1, v1 in json_Map[k0].items()} for k0 in json_Map.keys()}

    def physics(self):

        with open(self.PATH_PHYSICS, 'r') as Physics:
            json_Physics = json.load(Physics)

        return {k: v['Value'] for k, v in json_Physics.items()}

    def rocket(self):

        with open(self.PATH_ROCKET, 'r') as Rocket:
            json_Rocket = json.load(Rocket)

        return {k0: v0['Value'] if 'Value' in v0.keys() else {k1: v1['Value'] for k1, v1 in json_Rocket[k0].items()} for k0, v0 in json_Rocket.items()}

    def simulator(self):

        with open(self.PATH_SIMULATOR, 'r') as Simulator:
            json_Simulator = json.load(Simulator)

        return {k: v['Value'] for k, v in json_Simulator.items()}

    def thrust(self):

        with open(self.PATH_THRUST, 'r') as Thrust:
            json_Thrust = json.load(Thrust)

        return {k: v['Value'] for k, v in json_Thrust.items()}

    def wind(self):

        with open(self.PATH_WIND, 'r') as Wind:
            json_Wind = json.load(Wind)

        return {k0: {k1: v1['Value'] for k1, v1 in json_Wind[k0].items()} for k0 in json_Wind.keys()}


Constants = Constants()

if __name__ == '__main__':
    print(Constants.map())
    print(Constants.thrust())