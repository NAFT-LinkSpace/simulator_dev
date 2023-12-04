# mypy: ignore-errors
import pandas as pd
import numpy as np
from pathlib import Path

from IPython.display import display

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))

from pre_processing import PATH_RESULTS
PATH_HISTORIES = PATH_RESULTS + '/Histories'


#if __name__ == '__main__':
#    from pre_processing import main as main_pre



HEADER = {
    'Launcher':['Time [s]', 'Phase', 'DownRange [m]', 'r_x [m]', 'r_y [m]', 'r_z [m]', 'v_norm [m/s]', 'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]', 'a_norm [G]', 'a_x [G]', 'a_y [G]', 'a_z [G]',
                'Φ [deg]', 'Θ [deg]', 'Ψ [deg]', 'ω_x [rad/s]', 'ω_y [rad/s]', 'ω_z [m/s]', 'wind_norm [m/s]', 'wind_x [m/s]', 'wind_y [m/s]', 'wind_z [m/s]'],

    'Align':['Time [s]', 'Phase', 'v_norm [m/s]', 'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]', 'a_norm [G]', 'a_x [G]', 'a_y [G]', 'a_z [G]',
            'ω_x [rad/s]', 'ω_y [rad/s]', 'ω_z [m/s]', 'wind_norm [m/s]', 'wind_x [m/s]', 'wind_y [m/s]', 'wind_z [m/s]'],

    'GND':['Time [s]', 'Phase', 'DownRange [m]', 'r_x [m]', 'r_y [m]', 'r_z [m]', 'v_norm [m/s]', 'v_x [m/s]', 'v_y [m/s]', 'v_z [m/s]', 'a_norm [G]', 'a_x [G]', 'a_y [G]', 'a_z [G]', 'γ [deg]', 'ξ [deg]'],

    'GND_Wind':['Time [s]', 'Phase', 'Altitude [m]', 'v_norm [m/s]', 'v_North [m/s]', 'v_East [m/s]', 'Azimuth [deg]'],

    'Body':['Time [s]', 'Phase', 'ω_x [rad/s]', 'ω_y [rad/s]', 'ω_z [m/s]', 'Roll [deg]', 'airflow_norm [m/s]', 'airflow_x [m/s]', 'airflow_y [m/s]', 'airflow_z [m/s]', 'α [deg]', 'β [deg]', 'I_xx [kg*m^2]', 'I_yy [kg*m^2]', 'I_zz [kg*m^2]'],

    'Body_Force':['Time [s]', 'Phase', 'F_thrust_x [N]', 'Fn_norm [N]', 'Fn_x [N]', 'Fn_y [N]', 'Fn_z [N]', 'Ft_norm [N]', 'Ft_x [N]', 'Ft_y [N]', 'Ft_z [N]'],

    'Body_Torque':['Time [s]', 'Phase', 'Tn_x [N*m]', 'Tn_y [N*m]', 'Tn_z [N*m]', 'Tt_x [N*m]', 'Tt_y [N*m]', 'Tt_z [N*m]'],

    'Scalar':['Time [s]', 'Phase', 'DownRange [m]', 'Mass [kg]', 'Cg [m]', 'Cp [m]', 'Fst', 'air_density [kg/m^3]', 'dp [kPa]'],

    'Scalar_Aero':['Time [s]', 'Phase', 'Cl', 'Cd', 'Cn', 'Ct', 'α [deg]', 'β [deg]', 'γ [deg]', 'ξ [deg]']
}



test = np.array([0, 1, 2])
df = pd.DataFrame({'Time [s]': test})
display(df)

'''
def create_header(name):

    if __name__ == '__main__':
        main_pre()

    with pd.ExcelWriter(PATH_HISTORIES + '/' + str(name) + '.xlsx') as excel:

        for item in HEADER.items():
            pd.DataFrame(columns=item[1]).to_excel(excel, sheet_name=item[0], index=False, header=True)'''