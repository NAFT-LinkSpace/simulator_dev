# mypy: ignore-errors
import numpy as np
from numpy import  deg2rad, rad2deg, power, cross, sin, cos
from numpy.linalg import norm, inv

from scipy.integrate import solve_ivp
from findiff import FinDiff

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
    from for_main_processing import calculate_vectors as Vectors

else:
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from mypackages.for_pre_processing.input_from_Setting import Constants
    from . import for_calculate_variables as FCV
    from . import trans_coordinates as Trans
    from . import calculate_variables as CV
    from . import calculate_vectors as Vectors


TIME_STEP = Constants.simulator()['Time_Step']
TIME_MAX = Constants.simulator()['Time_Max']
LAUNCHER_LENGTH = Constants.launcher()['Length']
Cg_BEFORE = Constants.rocket()['Cg']['Before']
GAP = Constants.rocket()['Gap_Tail_Launcher']
OPEN_PARACHUTE = Constants.simulator()['Open_Parachute']
LAUNCHER_ANGLE = deg2rad(Constants.launcher()['Angle'])



def solve(ode, wind_std, wind_angle, X0, t_start, events):
    time_step = TIME_STEP
    t_max = TIME_MAX

    sol = solve_ivp(ode, (t_start, t_max), X0, args=(wind_std, wind_angle), dense_output=True, events=events)

    t_final = sol.t[-1]
    X_final = sol.y[:, -1]

    t = np.arange(t_start, t_final, time_step)
    Xt = sol.sol(t)

    return {'t_final' : t_final, 'X_final' : X_final, 't' : t, 'Xt' : Xt, 'fun_Xt' : sol.sol}



def launch_clear(t, X, wind_std, wind_angle):

    q0, q1, q2, q3 = X[0:4]
    q = np.quaternion(q0, q1, q2, q3)

    r_launcher = np.array(X[7:10])
    r_align = Trans.launcher_to_align(q, r_launcher)

    REACH_LINE = LAUNCHER_LENGTH + Cg_BEFORE

    return REACH_LINE - r_align[0]

launch_clear.terminal = True
launch_clear.direction = -1


def apogee(t, X, wind_std, wind_angle):

    vx_launcher = X[4] / CV.mass(t)

    return vx_launcher

apogee.terminal = True
apogee.direction = -1


def landing(t, X, wind_std, wind_angle):
    LANDING_LEVEL = Constants.simulator()['Landing_Level']
    SEA_LEVEL = - Constants.launcher()['Altitude']
    GROUND_LEVEL = 0

    if LANDING_LEVEL == 0:
        return X[7] - SEA_LEVEL

    if LANDING_LEVEL == 1:
        return X[7] - GROUND_LEVEL

landing.terminal = True
landing.direction = -1


X0_phase_1 = np.zeros(17)

X0_phase_1[0:4] = cos((LAUNCHER_ANGLE - np.pi / 2) / 2), 0, sin((LAUNCHER_ANGLE - np.pi / 2) / 2), 0
X0_phase_1[4:7] = 0, 0, 0
X0_phase_1[7:10] = (Cg_BEFORE + GAP) * sin(LAUNCHER_ANGLE), 0, (Cg_BEFORE + GAP) * cos(LAUNCHER_ANGLE)
X0_phase_1[10:13] = 0, 0, 0
X0_phase_1[13] = 0
X0_phase_1[14:17] = 0, -(90 - Constants.launcher()['Angle']), 0


def phase_1(t, X, wind_std, wind_angle):

    q0, q1, q2, q3 = X[0:4]
    px_launcher, py_launcher, pz_launcher = X[4:7]
    rx_launcher, ry_launcher, rz_launcher = X[7:10]
    ang_vx_align, ang_vy_align, ang_vz_align = X[10:13]
    roll_align = X[13]
    phi, theta, psi = X[14:17]


    diff_X = np.zeros_like(X)


    q = np.quaternion(q0, q1, q2, q3)

    ang_v_align = np.array([ang_vx_align, ang_vy_align, ang_vz_align])
    ang_v_launcher = Trans.align_to_launcher(q, ang_v_align)

    diff_roll = rad2deg(ang_v_align[0])
    diff_phi, diff_theta, diff_psi = rad2deg(ang_v_launcher)
    diff_q = Trans.diff_quaternion(q, ang_v_align)


    v_launcher = np.array([px_launcher, py_launcher, pz_launcher]) / CV.mass(t)
    diff_r_launcher = v_launcher

    wind_launcher = Vectors.wind_launcher(rx_launcher, wind_std, wind_angle)
    airflow_launcher = wind_launcher - v_launcher
    airflow_align = Trans.launcher_to_align(q, airflow_launcher)

    F_align = Trans.launcher_to_align(q, Vectors.gravity_launcher(t)) + Vectors.thrust_align(t) + Vectors.Ft_align(rx_launcher, airflow_align)

    if F_align[0] < 0:
        F_align = np.array([0, 0, 0])

    F_launcher = Trans.align_to_launcher(q, F_align)
    diff_p_launcher = F_launcher

    diff_ang_v_align = np.array([0, 0, 0])

    diff_X[0:4] = diff_q.components
    diff_X[4:7] = diff_p_launcher
    diff_X[7:10] = diff_r_launcher
    diff_X[10:13] = diff_ang_v_align
    diff_X[13] = diff_roll
    diff_X[14:17] = diff_phi, diff_theta, diff_psi

    return diff_X

def phase_2(t, X, wind_std, wind_angle):

    q0, q1, q2, q3 = X[0:4]
    px_launcher, py_launcher, pz_launcher = X[4:7]
    rx_launcher, ry_launcher, rz_launcher = X[7:10]
    ang_vx_align, ang_vy_align, ang_vz_align = X[10:13]
    roll_align = X[13]
    phi, theta, psi = X[14:17]

    #返す用のリスト作成
    diff_X = np.zeros_like(X)

    #クオータニオンを定義
    q = np.quaternion(q0, q1, q2, q3)

    ang_v_align = np.array([ang_vx_align, ang_vy_align, ang_vz_align])#地面から見た角速度
    #座標変換
    ang_v_launcher = Trans.align_to_launcher(q, ang_v_align)#ランチャーからみた角速度

    diff_roll = rad2deg(ang_v_align[0])
    diff_phi, diff_theta, diff_psi = rad2deg(ang_v_launcher)
    diff_q = Trans.diff_quaternion(q, ang_v_align)


    v_launcher = np.array([px_launcher, py_launcher, pz_launcher]) / CV.mass(t)
    diff_r_launcher = v_launcher

    wind_launcher = Vectors.wind_launcher(rx_launcher, wind_std, wind_angle)
    airflow_launcher = wind_launcher - v_launcher
    airflow_align = Trans.launcher_to_align(q, airflow_launcher)

    #並進の運動方程式
    #F=重力、推力、空気力１、空気力２　の４項の和
    F_launcher = Vectors.gravity_launcher(t) + Trans.align_to_launcher(q, Vectors.thrust_align(t) + Vectors.Ft_align(rx_launcher, airflow_align) + Vectors.Fn_align(rx_launcher, airflow_align))
    diff_p_launcher = F_launcher

    I = CV.inertia(t)
    diff_ang_v_align = inv(I) @ (Vectors.Tn_align(t, rx_launcher, airflow_align) - cross(ang_v_align, I @ ang_v_align))

    diff_X[0:4] = diff_q.components
    diff_X[4:7] = diff_p_launcher
    diff_X[7:10] = diff_r_launcher
    diff_X[10:13] = diff_ang_v_align
    diff_X[13] = diff_roll
    diff_X[14:17] = diff_phi, diff_theta, diff_psi

    return diff_X

def phase_3(t, X, wind_std, wind_angle):

    q0, q1, q2, q3 = X[0:4]
    px_launcher, py_launcher, pz_launcher = X[4:7]
    rx_launcher, ry_launcher, rz_launcher = X[7:10]
    ang_vx_align, ang_vy_align, ang_vz_align = X[10:13]
    roll_align = X[13]
    phi, theta, psi = X[14:17]


    diff_X = np.zeros_like(X)


    q = np.quaternion(q0, q1, q2, q3)

    v_launcher = np.array([px_launcher, py_launcher, pz_launcher]) / CV.mass(t)
    diff_r_launcher = v_launcher

    wind_launcher = Vectors.wind_launcher(rx_launcher, wind_std, wind_angle)

    airflow_launcher = wind_launcher - v_launcher
    airflow_align = Trans.parachute_launcher_to_align(v_launcher, wind_launcher, airflow_launcher)

    ang_v_align = np.array([ang_vx_align, ang_vy_align, ang_vz_align])
    ang_v_launcher = Trans.parachute_align_to_launcher(v_launcher, wind_launcher, ang_v_align)

    diff_roll = rad2deg(ang_v_align[0])
    diff_phi, diff_theta, diff_psi = rad2deg(ang_v_launcher)
    diff_q = Trans.diff_quaternion(q, ang_v_align)


    F_launcher = Vectors.gravity_launcher(t) + Trans.parachute_align_to_launcher(v_launcher, wind_launcher,  Vectors.Fd_parachute(rx_launcher, airflow_align))
    diff_p_launcher = F_launcher

    diff_ang_v_align = np.array([0, 0, 0])

    diff_X[0:4] = diff_q.components
    diff_X[4:7] = diff_p_launcher
    diff_X[7:10] = diff_r_launcher
    diff_X[10:13] = diff_ang_v_align
    diff_X[13] = diff_roll
    diff_X[14:17] = diff_phi, diff_theta, diff_psi

    return diff_X


def solve_all(wind_std, wind_angle):

    d_dt = FinDiff(0, TIME_STEP)
    
    #パラシュート解散なし
    if OPEN_PARACHUTE == 0:
        #launch clear計算
        solve_phase_1 = solve(phase_1, wind_std, wind_angle, X0_phase_1, 0, launch_clear)
        #解１
        t_start_phase_2 = solve_phase_1['t_final']
        X0_phase_2 = solve_phase_1['X_final']
        #着弾計算
        solve_phase_2 = solve(phase_2, wind_std, wind_angle, X0_phase_2, t_start_phase_2, landing)
        #解２
        t1, t2 = solve_phase_1['t'], solve_phase_2['t'][1:]
        Xt1, Xt2 = solve_phase_1['Xt'], solve_phase_2['Xt'][:, 1:]
        #2解を結合
        t = np.concatenate([t1, t2], axis=0)
        Xt = np.concatenate([Xt1, Xt2], axis=1)
        #quaternion配列に変換
        q = qtn.as_quat_array(np.array([Xt[0], Xt[1], Xt[2], Xt[3]]).T)

        ang_v_align = np.array([Xt[10], Xt[11], Xt[12]]).T
        ang_v_launcher = Trans.align_to_launcher(q, ang_v_align)
        ang_v_gnd = Trans.launcher_to_gnd(ang_v_launcher)

        roll = Xt[13]
        phi, theta, psi = Xt[14], Xt[15], Xt[16]

        r_launcher = np.array([Xt[7], Xt[8], Xt[9]]).T
        r_gnd = Trans.launcher_to_gnd(r_launcher)
        down_range = np.array([np.sqrt(r_launcher[i, 1]**2 + r_launcher[i, 2]**2) for i in range(len(r_launcher)) ])
        altitude = r_launcher[:, 0]

        v_launcher = np.array([Xt[4] / CV.mass(t), Xt[5] / CV.mass(t), Xt[6] / CV.mass(t)]).T
        v_align = Trans.launcher_to_align(q, v_launcher)
        v_gnd = Trans.launcher_to_gnd(v_launcher)
        v_norm = norm(v_launcher, axis=1)

        wind_launcher = Vectors.wind_launcher(altitude, wind_std, wind_angle)
        wind_align = Trans.launcher_to_align(q, wind_launcher)
        wind_gnd = Trans.launcher_to_gnd(wind_launcher)
        wind_north, wind_east = wind_gnd[:, 0], wind_gnd[:, 1]
        wind_norm = norm(wind_launcher, axis=1)

        WIND_MODEL = Constants.simulator()['Wind_Model']
        if WIND_MODEL == 0:
            wind_azimuth = FCV.Power.wind_azimuth(wind_angle)

        if WIND_MODEL == 1:
            wind_azimuth = FCV.Statistic.wind_azimuth(altitude)


        airflow_launcher = wind_launcher - v_launcher
        airflow_align = wind_align - v_align
        airflow_norm = norm(airflow_launcher, axis=1)

        Fn_align = Vectors.Fn_align(altitude, airflow_align)
        Fn_norm = norm(Fn_align, axis=1)
        Ft_align = Vectors.Ft_align(altitude, airflow_align)
        Ft_norm = norm(Ft_align, axis=1)
        Tn_align = Vectors.Tn_align(t, altitude, airflow_align)
        Tn_align_y = Tn_align[:, 1]

        g = Constants.physics()['g']
        a_launcher = (Vectors.gravity_launcher(t) + Trans.align_to_launcher(q, Fn_align + Ft_align)) / np.array([CV.mass(t) * g, CV.mass(t) * g, CV.mass(t) * g]).T
        a_align = Trans.launcher_to_align(q, a_launcher)
        a_gnd = Trans.launcher_to_gnd(a_launcher)
        a_norm = norm(a_launcher, axis=1)

        phase = np.concatenate([np.full_like(t1, 1), np.full_like(t2, 2)], axis=0)

        mass = CV.mass(t)
        I = CV.inertia(t)
        I_xx, I_yy, I_zz = I[:, 0, 0], I[:, 1, 1], I[:, 2, 2]

        thrust = FCV.Thrust.thrust(t)
        Cg = CV.Cg(t)

        alpha_airflow = CV.alpha_airflow(airflow_align)
        alpha = CV.alpha(airflow_align)
        beta = CV.beta(airflow_align)
        gamma = CV.gamma(v_gnd)
        xi = CV.xi(v_gnd)

        Cp = CV.Cp(alpha_airflow)
        Fst = CV.Fst(t, alpha_airflow)

        Cl = CV.Cl(alpha_airflow)
        Cn = CV.Cn(alpha_airflow)
        Cd = CV.Cd(alpha_airflow)
        Ct = CV.Ct(alpha_airflow)

        air_density = CV.air_density(altitude)
        dp = CV.dp_as_kPa(altitude, alpha_airflow)

        index_launch_clear = len(t1) - 1
        index_Max_Q = np.argmax(dp)
        index_apogee = np.argmax(altitude)
        index_landing = -1

        t_launch_clear, a_norm_launch_clear, airflow_norm_launch_clear = t[index_launch_clear], a_norm[index_launch_clear], airflow_norm[index_launch_clear]
        alpha_launch_clear, beta_launch_clear = alpha[index_launch_clear], beta[index_launch_clear]

        t_Max_Q, altitude_Max_Q, airflow_norm_Max_Q, dp_Max_Q = t[index_Max_Q], altitude[index_Max_Q], airflow_norm[index_Max_Q], dp[index_Max_Q]

        t_apogee, altitude_apogee, down_range_apogee, airflow_norm_apogee = t[index_apogee], altitude[index_apogee], down_range[index_apogee], airflow_norm[index_apogee]
        latitude_apogee = Trans.gnd_to_latlon(np.array([r_gnd[index_apogee, 0], r_gnd[index_apogee, 1]]))[0]
        longitude_apogee = Trans.gnd_to_latlon(np.array([r_gnd[index_apogee, 0], r_gnd[index_apogee, 1]]))[1]

        t_landing, down_range_landing = t[index_landing], down_range[index_landing]
        latitude_landing = Trans.gnd_to_latlon(np.array([r_gnd[index_landing, 0], r_gnd[index_landing, 1]]))[0]
        longitude_landing = Trans.gnd_to_latlon(np.array([r_gnd[index_landing, 0], r_gnd[index_landing, 1]]))[1]
    """
    #パラシュート頂点解散あり
    if OPEN_PARACHUTE == 1:
        #launch clear計算
        solve_phase_1 = solve(phase_1, wind_std, wind_angle, X0_phase_1, 0, launch_clear)
        #解１
        t_start_phase_2 = solve_phase_1['t_final']
        X0_phase_2 = solve_phase_1['X_final']
        #Apogee計算
        solve_phase_2 = solve(phase_2, wind_std, wind_angle, X0_phase_2, t_start_phase_2, apogee)
        #解２
        t_start_phase_3 = solve_phase_2['t_final']
        X0_phase_3 = solve_phase_2['X_final']
        #パラシュート解散
        solve_phase_3 = solve(phase_3, wind_std, wind_angle, X0_phase_3, t_start_phase_3, landing)

        #解を結合
        t = np.concatenate([solve_phase_1['t'], solve_phase_2['t'][1:], solve_phase_3['t'][1:]], axis=0)
        Xt = np.concatenate([solve_phase_1['Xt'], solve_phase_2['Xt'][:, 1:], solve_phase_3['Xt'][:, 1:]], axis=1)

        mass = np.concatenate([solve_phase_1['t'], solve_phase_2['t'][1:], solve_phase_3['t'][1:]], axis=0)
    """
    return {'Launcher' :
                {'time' : t, 'phase' : phase,
                'DownRange' : down_range, 'r_x' : r_launcher[:, 0], 'r_y' : r_launcher[:, 1], 'r_z' : r_launcher[:, 2],
                'v_norm' : v_norm, 'v_x' : v_launcher[:, 0], 'v_y' : v_launcher[:, 1], 'v_z' : v_launcher[:, 2],
                'a_norm' : a_norm, 'a_x' : a_launcher[:, 0], 'a_y' : a_launcher[:, 1], 'a_z' : a_launcher[:, 2],
                'phi' : phi, 'theta' : theta, 'psi' : psi,
                'omega_x' : ang_v_launcher[:, 0], 'omega_y' : ang_v_launcher[:, 1], 'omega_z' : ang_v_launcher[:, 2],
                'wind_norm' : wind_norm, 'wind_x' : wind_launcher[:, 0], 'wind_y' : wind_launcher[:, 1], 'wind_z' : wind_launcher[:, 2]},

            'Align' :
                {'time' : t, 'phase' : phase,
                'v_norm' : v_norm, 'v_x' : v_align[:, 0], 'v_y' : v_align[:, 1], 'v_z' : v_align[:, 2],
                'a_norm' : a_norm, 'a_x' : a_align[:, 0], 'a_y' : a_align[:, 1], 'a_z' : a_align[:, 2],
                'omega_x' : ang_v_align[:, 0], 'omega_y' : ang_v_align[:, 1], 'omega_z' : ang_v_align[:, 2],
                'wind_norm' : wind_norm, 'wind_x' : wind_align[:, 0], 'wind_y' : wind_align[:, 1], 'wind_z' : wind_align[:, 2]},

            'GND' :
                {'time' : t, 'phase' : phase,
                'DownRange' : down_range, 'r_x' : r_gnd[:, 0], 'r_y' : r_gnd[:, 1], 'r_z' : r_gnd[:, 2],
                'v_norm' : v_norm, 'v_x' : v_gnd[:, 0], 'v_y' : v_gnd[:, 1], 'v_z' : v_gnd[:, 2],
                'a_norm' : a_norm, 'a_x' : a_gnd[:, 0], 'a_y' : a_gnd[:, 1], 'a_z' : a_gnd[:, 2],
                'gamma' : gamma, 'xi' : xi},

            'GND_Wind' :
                {'time' : t, 'phase' : phase,
                'Altitude' : r_launcher[:, 0], 'wind_norm' : wind_norm, 'wind_north' : wind_north, 'wind_east' : wind_east, 'wind_azimuth' : wind_azimuth},

            'Body' :
                {'time' : t, 'phase' : phase,
                'omega_x' : ang_v_align[:, 0], 'omega_y' : ang_v_align[:, 1], 'omega_z' : ang_v_align[:, 2],
                'Roll' : roll,
                'airflow_norm' : airflow_norm, 'airflow_x' : airflow_align[:, 0], 'airflow_y' : airflow_align[:, 1], 'airflow_z' : airflow_align[:, 2],
                'alpha_airflow' : alpha_airflow, 'alpha' : alpha, 'beta' : beta,
                'I_xx' : I_xx, 'I_yy' : I_yy, 'I_zz' : I_zz},

            'Body_Force' :
                {'time' : t, 'phase' : phase,
                'thrust' : thrust, 'Fn' : Fn_norm, 'Ft' : Ft_norm},

            'Body_Torque' :
                {'Tn' : Tn_align_y},

            'Scalar' :
                {'time' : t, 'phase' : phase,
                'DownRange' : down_range, 'mass' : mass,
                'Cg' : Cg, 'Cp' : Cp, 'Fst' : Fst,
                'air_density' : air_density, 'dp' : dp},

            'Scalar_Aero' :
                {'time' : t, 'phase' : phase,
                'Cl' : Cl, 'Cd' : Cd, 'Cn' : Cn, 'Ct' : Ct,
                'alpha_airflow' : alpha_airflow, 'alpha' : alpha, 'beta' : beta, 'gamma' : gamma, 'xi' : xi},

            'Summary' :
                {'Wind' :
                    {'wind' : wind_std, 'angle' : wind_angle},

                'Launch_Clear' :
                    {'time' : t_launch_clear, 'a_norm' : a_norm_launch_clear, 'airflow_norm' : airflow_norm_launch_clear,
                    'alpha' : alpha_launch_clear, 'beta' : beta_launch_clear},

                'Max_Q' :
                    {'time' : t_Max_Q, 'altitude' : altitude_Max_Q, 'airflow_norm' : airflow_norm_Max_Q, 'dp' : dp_Max_Q},

                'Apogee' :
                    {'time' : t_apogee, 'altitude' : altitude_apogee, 'DownRange' : down_range_apogee, 'airflow_norm' : airflow_norm_apogee,
                    'longitude' : longitude_apogee, 'latitude' : latitude_apogee},

                'Landing' :
                    {'time' : t_landing, 'DownRange' : down_range_landing, 'longitude' : longitude_landing, 'latitude' : latitude_landing}}}
"""
for wind_std in range (8):
    for wind_angle in np.linspace(0, 360, 9)[:-2]:
        print(solve_all(wind_std, wind_angle)['Summary']['Landing']['longitude'])
        print(solve_all(wind_std, wind_angle)['Summary']['Landing']['latitude'])
"""


if __name__ == '__main__':
    import pandas as pd
    from openpyxl import load_workbook
    from openpyxl.styles import Font

    # Call the solve_all function
    output = solve_all(3, 180)

    # Create a Pandas Excel writer using XlsxWriter as the engine
    with pd.ExcelWriter('output.xlsx', engine='openpyxl') as writer:
        for key, value in output.items():
            # Skip the 'Summary' key for the Excel file
            if key == 'Summary':
                continue

            # Convert the dictionary to a DataFrame
            df = pd.DataFrame(value)

            # Write the DataFrame to an Excel sheet with the name of the key, without the index
            df.to_excel(writer, sheet_name=key, index=False)


    def flatten_dict(dd, separator='_', prefix=''):
        return { prefix + separator + k if prefix else k : v
                 for kk, vv in dd.items()
                 for k, v in flatten_dict(vv, separator, kk).items()
                 } if isinstance(dd, dict) else { prefix : dd }
    
    # Flatten the 'Summary' data
    flat_summary = flatten_dict(output['Summary'])

    # Convert the flattened data to a DataFrame
    summary_df = pd.DataFrame(flat_summary, index=[0])

    # Write the DataFrame to a CSV file
    summary_df.to_csv('summary.csv', index=False)
