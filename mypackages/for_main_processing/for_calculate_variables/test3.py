from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent.parent))

    from for_main_processing.for_calculate_variables.calculate_thrust import Thrust_Interpolated as Thrust
    from for_main_processing.for_calculate_variables.calculate_wind import Statistic_Wind_Interpolated as Statistic
    from for_main_processing.for_calculate_variables.calculate_wind import Power_Law_Wind as Power

print(Thrust.thrust(1.0))
print(Statistic.wind_norm(8.0))
print(Power.wind_norm(8.0, 3.0))

