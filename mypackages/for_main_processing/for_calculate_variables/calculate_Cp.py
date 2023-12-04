# mypy: ignore-errors
import numpy as np

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
Cp_ALPHA_0 = Constants.rocket()['Cp']['Alpha_0']
Cp_ALPHA_MAX = Constants.rocket()['Cp']['Alpha_Max']


def Cp(alpha):

    Cp_interpolated = interp1d([0, ALPHA_MAX], [Cp_ALPHA_0, Cp_ALPHA_MAX], bounds_error=False, fill_value=(Cp_ALPHA_0, Cp_ALPHA_MAX))
    Cp = Cp_interpolated(np.abs(alpha))

    return Cp


if __name__ == '__main__':

    alpha = np.linspace(-15, 15, num=100)

    Cp = Cp(alpha)

    fig, ax = plt.subplots(nrows=1, ncols=1)

    ax.plot(alpha, Cp, 'b', label='Cp [m]')

    ax.set_xlabel('Alpha [deg]')
    ax.grid()
    handler1, label1 = ax.get_legend_handles_labels()
    ax.legend(handler1, label1, loc='lower right')

    plt.show()