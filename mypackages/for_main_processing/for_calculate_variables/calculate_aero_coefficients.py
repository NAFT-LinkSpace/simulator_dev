# mypy: ignore-errors
import numpy as np
from numpy import sin, cos, deg2rad

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


ALPHA_MAX = Constants.rocket()['Alpha_Max']
Cl_ALPHA = Constants.rocket()['Cl_Alpha']
Cn_ALPHA = Constants.rocket()['Cn_Alpha']

def Cl(alpha):

    Cl_interpolated = interp1d([0, ALPHA_MAX], [0, Cl_ALPHA * deg2rad(ALPHA_MAX)], bounds_error=False, fill_value=(0, Cl_ALPHA * deg2rad(ALPHA_MAX)))
    Cl = Cl_interpolated(np.abs(alpha))

    return Cl

def Cn(alpha):

    Cn_interpolated = interp1d([0, ALPHA_MAX], [0, Cn_ALPHA * deg2rad(ALPHA_MAX)], bounds_error=False, fill_value=(0, Cn_ALPHA * deg2rad(ALPHA_MAX)))
    Cn = Cn_interpolated(np.abs(alpha))

    return Cn


Cd0 = Constants.rocket()['Cd0']
Cd_Cl = Constants.rocket()['Coefficient_Cd_Induced']

Ct0 = Constants.rocket()['Ct0']
Ct_Cn = Constants.rocket()['Coefficient_Ct_Induced']

def Cd(alpha):

    Cd = Cd0 + Cd_Cl * Cl(alpha)**2

    return Cd

def Ct(alpha):

    Ct = Ct0 + Ct_Cn * Cn(alpha)**2

    return Ct



if __name__ == '__main__':

    alpha = np.linspace(-15, 15, num=100)

    var_Cl = Cl(alpha)
    var_Cn = Cn(alpha)
    var_Cd = Cd(alpha)
    var_Ct = Ct(alpha)

    fig, axes = plt.subplots(nrows=2, ncols=2)

    axes[0, 0].plot(alpha, var_Cl, 'r', label='Cl')

    axes[0, 0].set_xlabel('Alpha [deg]')
    axes[0, 0].grid()
    handler1, label1 = axes[0, 0].get_legend_handles_labels()
    axes[0, 0].legend(handler1, label1, loc='lower right')


    axes[0, 1].plot(alpha, var_Cn, 'g', label='Cn')

    axes[0, 1].set_xlabel('Alpha [deg]')
    axes[0, 1].grid()
    handler1, label1 = axes[0, 1].get_legend_handles_labels()
    axes[0, 1].legend(handler1, label1, loc='lower right')


    axes[1, 0].plot(alpha, var_Cd, 'b', label='Cd')

    axes[1, 0].set_xlabel('Alpha [deg]')
    axes[1, 0].grid()
    handler1, label1 = axes[1, 0].get_legend_handles_labels()
    axes[1, 0].legend(handler1, label1, loc='lower right')


    axes[1, 1].plot(alpha, var_Ct, 'gold', label='Ct')

    axes[1, 1].set_xlabel('Alpha [deg]')
    axes[1, 1].grid()
    handler1, label1 = axes[1, 1].get_legend_handles_labels()
    axes[1, 1].legend(handler1, label1, loc='lower right')


    plt.show()