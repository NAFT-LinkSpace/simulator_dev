# mypy: ignore-errors
import numpy as np
from numpy import  rad2deg, power, cross
from numpy.linalg import norm

import quaternion as qtn

if __name__ == '__main__':
    import matplotlib.pyplot as plt

from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from for_pre_processing.input_from_Setting import Constants
    from for_main_processing import for_calculate_variables as FCV
    from for_main_processing import trans_coordinates as Trans
    from for_main_processing import calculate_variables as CV

else:
    from for_pre_processing.input_from_Setting import Constants
    from . import for_calculate_variables as FCV
    from . import trans_coordinates as Trans
    from . import calculate_variables as CV


WIND_MODEL = Constants.simulator()['Wind_Model']

def wind_launcher(altitude, wind_std, wind_angle):
    if WIND_MODEL == 0:

        wind_launcher_x = np.zeros_like(altitude)
        wind_launcher_y = FCV.Power.wind_launcher_y(altitude, wind_std, wind_angle)
        wind_launcher_z = FCV.Power.wind_launcher_z(altitude, wind_std, wind_angle)

        wind_launcher = np.array([wind_launcher_x, wind_launcher_y, wind_launcher_z]).T

    if WIND_MODEL == 1:
        wind_launcher = np.array([np.zeros_like(altitude), FCV.Statistic.wind_launcher_y(altitude), FCV.Statistic.wind_launcher_z(altitude)]).T

    return wind_launcher


g = Constants.physics()['g']

def gravity_launcher(time):

    gravity_launcher = np.array([-CV.mass(time) * g, np.zeros_like(time), np.zeros_like(time)]).T

    return gravity_launcher

def thrust_align(time):

    thrust_align = np.array([CV.thrust(time), np.zeros_like(time), np.zeros_like(time)]).T

    return thrust_align

def Cp_body(time, airflow_align):

    alpha = CV.alpha_airflow(airflow_align)

    Cp = np.array([CV.Cp(alpha) - CV.Cg(time), np.zeros_like(time), np.zeros_like(time)]).T

    return Cp

D_BODY = Constants.rocket()['Body_Diameter']
S_BODY = np.pi * (D_BODY / 2)**2

def Fn_align(altitude, airflow_align):

    alpha = CV.alpha_airflow(airflow_align)
    #alpha = CV.alpha(airflow_align)
    ex = np.array([1, 0, 0])
    airflow_yz = cross(cross(ex, airflow_align), ex)

    if airflow_align.ndim > 1:

        direction_yz = np.array([airflow_yz[i] / norm(airflow_yz, axis=1)[i] if norm(airflow_yz, axis=1)[i] != 0 else np.array([0, 0, 0]) for i in range(len(airflow_yz))])
        Fn_align_yz = np.array([CV.dp(altitude, airflow_align)[i] * S_BODY * CV.Cn(alpha)[i] * direction_yz[i] for i in range(len(airflow_yz))])

    else:

        if norm(airflow_yz) != 0:
            direction_yz = airflow_yz / norm(airflow_yz)
        else:
            direction_yz = np.array([0, 0, 0])

        Fn_align_yz = CV.dp(altitude, airflow_align) * S_BODY * CV.Cn(alpha) * direction_yz

    return Fn_align_yz

def Tn_align(time, altitude, airflow_align):

    alpha = CV.alpha_airflow(airflow_align)
    #alpha = CV.alpha(airflow_align)
    Cp = Cp_body(time, airflow_align)
    Fn = Fn_align(altitude, airflow_align)

    Tn_align = cross(Cp, Fn)

    return Tn_align


def Ft_align(altitude, airflow_align):

    alpha = CV.alpha_airflow(airflow_align)
    #alpha = CV.alpha(airflow_align)

    if airflow_align.ndim > 1:
        airflow_x = np.array([airflow_align[:, 0], np.zeros_like(altitude), np.zeros_like(altitude)]).T
        direction_x = np.array([airflow_x[i] / norm(airflow_x, axis=1)[i] if norm(airflow_x, axis=1)[i] != 0 else np.array([0, 0, 0]) for i in range(len(airflow_x))])

        Ft_align_x = np.array([CV.dp(altitude, airflow_align)[i] * S_BODY * CV.Ct(alpha)[i] * direction_x[i] for i in range(len(airflow_x))])

    else:
        airflow_x = np.array([airflow_align[0], 0, 0])

        if norm(airflow_x) != 0:

            direction_x = airflow_x / norm(airflow_x)
        else:
            direction_x = np.array([0, 0, 0])

        Ft_align_x = CV.dp(altitude, airflow_align) * S_BODY * CV.Ct(alpha) * direction_x

    return Ft_align_x


V_PARACHUTE = Constants.rocket()['V_Parachute']
MASS_AFTER = Constants.rocket()['Mass']['After']

def Fd_parachute(altitude, airflow_align):

    rho0 = 1.225
    CdS = 2 * MASS_AFTER * g / (rho0 * V_PARACHUTE**2)

    if airflow_align.ndim > 1:
        airflow_x = np.array([airflow_align[:, 0], np.zeros_like(altitude), np.zeros_like(altitude)]).T
        direction_x = np.array([airflow_x[i] / norm(airflow_x, axis=1)[i] if norm(airflow_x, axis=1)[i] != 0 else np.array([0, 0, 0]) for i in range(len(airflow_x))])

        Fd_parachute = np.array([CV.dp(altitude, airflow_align)[i] * CdS * direction_x[i] for i in range(len(airflow_x))])

    else:
        airflow_x = np.array([airflow_align[0], 0, 0])

        if norm(airflow_x) != 0:

            direction_x = airflow_x / norm(airflow_x)
        else:
            direction_x = np.array([0, 0, 0])

        Fd_parachute = CV.dp(altitude, airflow_align) * CdS * direction_x

    return Fd_parachute



if __name__ == '__main__':

    alt = np.linspace(0, 10000, num=100)
    time = np.linspace(0, 15, num=100)
    alpha = np.linspace(0, 10, num=100)
    q = np.quaternion(1, 0, 0, 0)
    var_alpha = np.linspace(0, 15, num=100)

    time = 0
    airflow_align = np.array([-10, 0, 10])
    altitude = 0

    #print(Fd_parachute(altitude, airflow_align))
    #print(Fd_parachute(100, np.array([-100, 0, 0])))
    #print(CV.alpha_airflow(airflow_align))
    print(Tn_align(time, altitude, airflow_align))
