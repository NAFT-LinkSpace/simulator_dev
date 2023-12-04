# mypy: ignore-errors
import numpy as np
from numpy import sin, cos, deg2rad, power, mod

import pandas as pd
from scipy.interpolate import interp1d

if __name__ == '__main__':
    import matplotlib.pyplot as plt

from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent.parent))
    from for_pre_processing.input_from_Setting import Constants

else:
    from for_pre_processing.input_from_Setting import Constants


PATH_STATISTIC_WIND = Constants.wind()['Statistic']['Path_XLSX']
LAUNCHER_AZIMUTH = Constants.launcher()['Azimuth']


STATISTIC_WIND_pd = pd.read_excel(PATH_STATISTIC_WIND, sheet_name='Statistic_Wind').loc[:, ['alt(高度)', 'vel(風速)', 'dir(風向)']].dropna(how='all')
STATISTIC_WIND = STATISTIC_WIND_pd.to_numpy()


class Statistic_Wind_Interpolated:

    def __init__(self, wind, launcher_azimuth):

        self.Wind_Data = wind
        self.Launcher_Azimuth = launcher_azimuth

        self.Wind_Interpolated = interp1d(wind[:, 0], np.array([wind[:, 1], wind[:, 2]]), bounds_error=False, fill_value=((wind[0, 1], wind[0, 2]), (wind[-1, 1], wind[-1, 2])))

    def wind_norm(self, altitude):

        wind_norm = self.Wind_Interpolated(altitude)[0]

        return wind_norm

    def wind_azimuth(self, altitude):

        wind_azimuth = self.Wind_Interpolated(altitude)[1]

        return wind_azimuth

    def wind_north(self, altitude):

        wind_north = -self.wind_norm(altitude) * cos(deg2rad(self.wind_azimuth(altitude)))

        return wind_north

    def wind_east(self, altitude):

        wind_east = -self.wind_norm(altitude) * sin(deg2rad(self.wind_azimuth(altitude)))

        return wind_east

    def wind_launcher_z(self, altitude):

        wind_launcher_z = self.wind_north(altitude) * cos(deg2rad(self.Launcher_Azimuth)) + self.wind_east(altitude) * sin(deg2rad(self.Launcher_Azimuth))

        return wind_launcher_z

    def wind_launcher_y(self, altitude):

        wind_launcher_y = -self.wind_north(altitude) * sin(deg2rad(self.Launcher_Azimuth)) + self.wind_east(altitude) * cos(deg2rad(self.Launcher_Azimuth))

        return wind_launcher_y


HEIGHT_STD = Constants.wind()['Power_law']['Height_STD']
WIND_POW = Constants.wind()['Power_law']['Pow']


class Power_Law_Wind:
    def __init__ (self, height_std, wind_pow, launcher_azimuth):

        self.Height_STD = height_std
        self.Wind_Pow = wind_pow
        self.Launcher_Azimuth = launcher_azimuth

    def wind_norm(self, altitude, wind_std):

        wind_norm = wind_std * power((altitude + np.abs(altitude)) / (2 * self.Height_STD), self.Wind_Pow)

        return wind_norm

    def wind_azimuth(self, wind_angle):

        wind_azimuth = mod(np.array(wind_angle) + self.Launcher_Azimuth, 360)

        return wind_azimuth

    def wind_launcher_z(self, altitude, wind_std, wind_angle):

        wind_launcher_z = -self.wind_norm(altitude, wind_std) * cos(deg2rad(wind_angle))

        return wind_launcher_z

    def wind_launcher_y(self, altitude, wind_std, wind_angle):

        wind_launcher_y = -self.wind_norm(altitude, wind_std) * sin(deg2rad(wind_angle))

        return wind_launcher_y


Statistic_Wind_Interpolated = Statistic_Wind_Interpolated(STATISTIC_WIND, LAUNCHER_AZIMUTH)
Power_Law_Wind = Power_Law_Wind(HEIGHT_STD, WIND_POW, LAUNCHER_AZIMUTH)


if __name__ == '__main__':

    print(Power_Law_Wind.wind_azimuth([0, 180]))

    alt = np.linspace(-1000, 10000, num=100)

    wind_norm = Statistic_Wind_Interpolated.wind_norm(alt)
    wind_north = Statistic_Wind_Interpolated.wind_north(alt)
    wind_east = Statistic_Wind_Interpolated.wind_east(alt)
    wind_azimuth = Statistic_Wind_Interpolated.wind_azimuth(alt)

    wind_launcher_z = Statistic_Wind_Interpolated.wind_launcher_z(alt)
    wind_launcher_y = Statistic_Wind_Interpolated.wind_launcher_y(alt)

    wind_norm_power = Power_Law_Wind.wind_norm(alt, Statistic_Wind_Interpolated.wind_norm(HEIGHT_STD))
    wind_launcher_z_power = Power_Law_Wind.wind_launcher_z(alt, Statistic_Wind_Interpolated.wind_norm(HEIGHT_STD), Statistic_Wind_Interpolated.wind_azimuth(alt) - LAUNCHER_AZIMUTH)
    wind_launcher_y_power = Power_Law_Wind.wind_launcher_y(alt, Statistic_Wind_Interpolated.wind_norm(HEIGHT_STD), Statistic_Wind_Interpolated.wind_azimuth(alt) - LAUNCHER_AZIMUTH)


    fig, axes = plt.subplots(nrows=2, ncols=2)

    axes[0,0].plot(alt, wind_norm, 'black', label='Wind_norm [m/s]')
    axes[0,0].plot(alt, wind_north, 'r', label='Wind_north [m/s]')
    axes[0,0].plot(alt, wind_east, 'b', label='Wind_east [m/s]')

    axes[0,0].set_xlabel('Altitude [m]')
    axes[0,0].grid()
    handler1, label1 = axes[0,0].get_legend_handles_labels()
    axes[0,0].legend(handler1, label1, loc='lower right')


    axes[0,1].plot(alt, wind_azimuth, 'g', label='Wind_Azimuth [deg]')

    axes[0,1].set_xlabel('Altitude [m]')
    axes[0,1].grid()
    handler2, label2 = axes[0,1].get_legend_handles_labels()
    axes[0,1].legend(handler2, label2, loc='lower right')


    axes[1,0].plot(alt, wind_norm, 'black', label='Wind_norm [m/s]')
    axes[1,0].plot(alt, wind_launcher_z, 'r', label='Wind_z [m/s]')
    axes[1,0].plot(alt, wind_launcher_y, 'b', label='Wind_y [m/s]')

    axes[1,0].set_xlabel('Altitude [m]')
    axes[1,0].grid()
    handler1, label1 = axes[1,0].get_legend_handles_labels()
    axes[1,0].legend(handler1, label1, loc='lower right')


    axes[1,1].plot(alt, wind_norm_power, 'black', label='Wind_norm [m/s]')
    axes[1,1].plot(alt, wind_launcher_z_power, 'r', label='Wind_z [m/s]')
    axes[1,1].plot(alt, wind_launcher_y_power, 'b', label='Wind_y [m/s]')

    axes[1,1].set_xlabel('Altitude [m]')
    axes[1,1].grid()
    handler1, label1 = axes[1,1].get_legend_handles_labels()
    axes[1,1].legend(handler1, label1, loc='lower right')


    plt.show()
