# mypy: ignore-errors
import numpy as np
import pandas as pd
from scipy.integrate import trapezoid, cumulative_trapezoid
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


PATH_THRUST = Constants.thrust()['Path_XLSX']

THRUST_pd = pd.read_excel(PATH_THRUST, sheet_name='K240').loc[:, ['Time [s]', 'Thrust [N]']].dropna(how='all')
THRUST = THRUST_pd.to_numpy()


class Thrust_Interpolated:
    def __init__(self, thrust):

        self.Thrust_Data = thrust
        #線形補完
        self.Thrust_Interpolated = interp1d(thrust[:, 0], thrust[:, 1], bounds_error=False, fill_value=(0.0, 0.0))
        #補完した関数を積分
        self.Total_Impulse = trapezoid(thrust[:, 1], thrust[:, 0])
        #正規化した推力を補間
        self.Normalized_Impulse = cumulative_trapezoid(thrust[:, 1]/self.Total_Impulse, thrust[:, 0], initial=0)
        #正規化したものもう一度線形補間
        self.Normalized_Impulse_Interpolated = interp1d(thrust[:, 0], self.Normalized_Impulse, bounds_error=False, fill_value=(0, 1))
        #初期と最終データ線形に結んだ関数を作成
        self.linear_change_Interpolated = interp1d(np.array([thrust[0, 0], thrust[-1, 0]]), np.array([0, 1]), bounds_error=False, fill_value=(0, 1))

    def thrust(self, time):

        thrust = self.Thrust_Interpolated(time)

        return thrust

    def normalized_impulse(self, time):

        normalized_impulse = self.Normalized_Impulse_Interpolated(time)

        return normalized_impulse

    def linear_change(self, time):

        linear_change = self.linear_change_Interpolated(time)

        return linear_change


Thrust_Interpolated = Thrust_Interpolated(THRUST)


if __name__ == '__main__':

    t = np.linspace(0, 10, num=100)
    thrust = Thrust_Interpolated.thrust(t)
    normalized_impulse = Thrust_Interpolated.normalized_impulse(t)
    linear_change = Thrust_Interpolated.linear_change(t)


    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(t, thrust, 'r', label='Thrust [N]')
    ax2.plot(t, normalized_impulse, 'b', label='Normalized_Impulse')
    ax2.plot(t, linear_change, 'g', label='Linear_Change')

    handler1, label1 = ax1.get_legend_handles_labels()
    handler2, label2 = ax2.get_legend_handles_labels()
    ax1.legend(handler1 + handler2, label1 + label2, loc='upper center', borderaxespad=-3.5)

    ax1.grid()
    plt.show()
