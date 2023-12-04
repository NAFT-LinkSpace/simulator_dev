# mypy: ignore-errors
import numpy as np
from numpy import  rad2deg, power
from numpy.linalg import norm

if __name__ == '__main__':
    import matplotlib.pyplot as plt

from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from for_pre_processing.input_from_Setting import Constants
    from for_main_processing import for_calculate_variables as FCV

else:
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from for_pre_processing.input_from_Setting import Constants
    from . import for_calculate_variables as FCV


def thrust(time):

    thrust = FCV.Thrust.thrust(time)

    return thrust

def normalized_impulse(time):

    normalized_impulse = FCV.Thrust.normalized_impulse(time)

    return normalized_impulse

def linear_change(time):

    linear_change = FCV.Thrust.linear_change(time)

    return linear_change


def mass(time):

    mass_before = Constants.rocket()['Mass']['Before']
    mass_after = Constants.rocket()['Mass']['After']

    mass = (1 - normalized_impulse(time)) * mass_before + normalized_impulse(time) * mass_after

    return mass

def inertia(time):

    inertia_before = np.array(Constants.rocket()['Inertia']['Before'])
    inertia_after = np.array(Constants.rocket()['Inertia']['After'])

    if isinstance(time, np.ndarray):
        inertia = np.reshape((1 - normalized_impulse(time)), (-1, 1, 1)) * inertia_before + np.reshape(normalized_impulse(time), (-1, 1, 1)) * inertia_after

    else:
        inertia = (1 - normalized_impulse(time)) * inertia_before + normalized_impulse(time) * inertia_after

    return inertia

def Cg(time):

    Cg_before = Constants.rocket()['Cg']['Before']
    Cg_after = Constants.rocket()['Cg']['After']

    Cg = (1 - normalized_impulse(time)) * Cg_before + normalized_impulse(time) * Cg_after

    return Cg


def Cp(alpha):

    return FCV.Cp(alpha)

def Fst(time, alpha):

    L = Constants.rocket()['Body_Length']

    Fst = 100 * (Cg(time) - Cp(alpha)) / L

    return Fst


def Cl(alpha):

    return FCV.Aero.Cl(alpha)

def Cn(alpha):

    return FCV.Aero.Cn(alpha)

def Cd(alpha):

    return FCV.Aero.Cd(alpha)

def Ct(alpha):

    return FCV.Aero.Ct(alpha)


def air_density(altitude):

    R = Constants.physics()['R']
    P_STD = Constants.physics()['Pressure_STD']
    T_STD = Constants.physics()['Temperature_STD']

    air_density = (288.15**(-5.256) * P_STD / R) * power((T_STD + 273.15 - 6.5 * ((altitude + np.abs(altitude)) / (2 * 1000))), 4.256)

    return air_density

def dp(altitude, airflow):

    if airflow.ndim > 1:
        dp = (1/2) * air_density(altitude) * norm(airflow, axis=1)**2

    else:
        dp = (1/2) * air_density(altitude) * norm(airflow)**2

    return dp

def dp_as_kPa(altitude, airflow):

    return dp(altitude, airflow) / 1000

def alpha_airflow(airflow):

    if airflow.ndim > 1:
        alpha_airflow = np.arccos(-airflow[:, 0] / (norm(airflow, axis=1) + 1e-6))

    else:
        alpha_airflow = np.arccos(-airflow[0] / (norm(airflow) + 1e-6))

    return rad2deg(alpha_airflow)

def alpha(airflow):

    if airflow.ndim > 1:
        alpha = np.arctan(-airflow[:, 2] / (-airflow[:, 0] + 1e-6))

    else:
        alpha = np.arctan(-airflow[2] / (-airflow[0] + 1e-6))

    return rad2deg(alpha)

def beta(airflow):

    if airflow.ndim > 1:
        beta = np.arcsin(-airflow[:, 1] / (norm(airflow, axis=1) + 1e-6))

    else:
        beta = np.arcsin(-airflow[1] / (norm(airflow) + 1e-6))

    return rad2deg(beta)

def gamma(velocity_Cg):

    if velocity_Cg.ndim > 1:
        gamma = np.arctan(-velocity_Cg[:, 2] / (np.sqrt(velocity_Cg[:, 0]**2 + velocity_Cg[:, 1]**2) + 1e-6))

    else:
        gamma = np.arctan(-velocity_Cg[2] / (np.sqrt(velocity_Cg[0]**2 + velocity_Cg[1]**2) + 1e-6))

    return rad2deg(gamma)

def xi(velocity_Cg):

    if velocity_Cg.ndim > 1:
        xi = np.arctan(velocity_Cg[:, 1] / (velocity_Cg[:, 0] + 1e-6))

    else:
        xi = np.arctan(velocity_Cg[1] / (velocity_Cg[0] + 1e-6))

    return rad2deg(xi)


if __name__ == '__main__':

    print(dp(1000, np.array([100, 0, 0])))
    print(alpha_airflow(np.array([0, 0, 100])))
    print(xi(np.array([[100, 100, 0], [100, 0, 100]])))

    alt = np.linspace(-1000, 10000, num=100)
    var_alpha = np.linspace(-15, 15, num=100)
    time = np.linspace(0, 15, num=100)

    var_air_density = air_density(alt)
    var_Cg = Cg(time)
    var_mass = mass(time)
    var_Ixx, var_Iyy, var_Izz = inertia(time)[:, 0, 0], inertia(time)[:, 0, 0], inertia(time)[:, 2, 2]

    var_Cl = Cl(var_alpha)
    var_Cn = Cn(var_alpha)
    var_Cd = Cd(var_alpha)
    var_Ct = Ct(var_alpha)

    fig, axes = plt.subplots(nrows=2, ncols=3)

    axes[0, 0].plot(alt, var_air_density, 'b', label='rho [kg/m^3]')

    axes[0, 0].set_xlabel('Altitude [m]')
    axes[0, 0].grid()
    handler1, label1 = axes[0, 0].get_legend_handles_labels()
    axes[0, 0].legend(handler1, label1, loc='upper right')


    axes[0, 1].plot(time, var_Cg, 'r', label='Cg [m]')

    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].grid()
    handler1, label1 = axes[0, 1].get_legend_handles_labels()
    axes[0, 1].legend(handler1, label1, loc='lower right')


    axes[1, 0].plot(time, var_mass, 'black', label='mass [kg]')

    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].grid()
    handler1, label1 = axes[1, 0].get_legend_handles_labels()
    axes[1, 0].legend(handler1, label1, loc='upper right')


    axes[1, 1].plot(time, var_Ixx, 'r', label='Ixx [kg*m^2]')
    axes[1, 1].plot(time, var_Iyy, 'b', label='Iyy [kg*m^2]')
    axes[1, 1].plot(time, var_Izz, 'g', label='Izz [kg*m^2]')

    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].grid()
    handler1, label1 = axes[1, 1].get_legend_handles_labels()
    axes[1, 1].legend(handler1, label1, loc='lower right')


    axes[0, 2].plot(var_alpha, var_Cl, 'r', label='Cl')
    axes[0, 2].plot(var_alpha, var_Cn, 'g', label='Cn')

    axes[0, 2].set_xlabel('Alpha [deg]')
    axes[0, 2].grid()
    handler1, label1 = axes[0, 2].get_legend_handles_labels()
    axes[0, 2].legend(handler1, label1, loc='lower right')


    axes[1, 2].plot(var_alpha, var_Cd, 'b', label='Cd')
    axes[1, 2].plot(var_alpha, var_Ct, 'gold', label='Ct')

    axes[1, 2].set_xlabel('Alpha [deg]')
    axes[1, 2].grid()
    handler1, label1 = axes[1, 2].get_legend_handles_labels()
    axes[1, 2].legend(handler1, label1, loc='lower right')


    plt.show()

