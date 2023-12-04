#シミュレーターの簡易実行用スクリプト
import numpy as np
from numpy import  deg2rad, rad2deg, power, cross, sin, cos
from numpy.linalg import norm, inv

from scipy.integrate import solve_ivp
from itertools import product

import quaternion as qtn
from pprint import pprint

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
    from for_pre_processing.input_from_Setting import Constants
    from . import for_calculate_variables as FCV
    from . import trans_coordinates as Trans
    from . import calculate_variables as CV
    from . import calculate_vectors as Vectors

#パスの設定
TIME_STEP = Constants.simulator()['Time_Step']
TIME_MAX = Constants.simulator()['Time_Max']

#各種定数の設定
LAUNCHER_LENGTH = Constants.launcher()['Length']
Cg_BEFORE = Constants.rocket()['Cg']['Before']
GAP = Constants.rocket()['Gap_Tail_Launcher']
LAUNCHER_ANGLE = deg2rad(Constants.launcher()['Angle'])


def solve(ode, wind_std, wind_angle, X0, t_start, events):
    """
    Solves the given ordinary differential equation (ODE) with the provided initial conditions and events.

    Parameters:
    ode (function): The ODE to be solved.
    wind_std (float): Standard deviation of the wind.
    wind_angle (float): Angle of the wind in degrees.
    X0 (array): Initial conditions.
    t_start (float): Start time.
    events (list): Events to track during the solution of the ODE.

    Returns:
    dict: A dictionary containing the final time and state, the time and state arrays, and the solution function.
    """
    time_step = TIME_STEP
    t_max = TIME_MAX

    sol = solve_ivp(ode, (t_start, t_max), X0, args=(wind_std, wind_angle), dense_output=True, events=events)

    t_final = sol.t[-1]
    X_final = sol.y[:, -1]

    t = np.arange(t_start, t_final, time_step)
    Xt = sol.sol(t)

    return {'t_final' : t_final, 'X_final' : X_final, 't' : t, 'Xt' : Xt, 'fun_Xt' : sol.sol}


#計算の終了条件
def launch_clear(t, X, wind_std, wind_angle):

    q0, q1, q2, q3 = X[0:4]
    q = np.quaternion(q0, q1, q2, q3)

    r_launcher = np.array(X[7:10])
    r_align = Trans.launcher_to_align(q, r_launcher)

    REACH_LINE = LAUNCHER_LENGTH + Cg_BEFORE

    return REACH_LINE - r_align[0]

launch_clear.terminal = True#終了条件を割り当て
launch_clear.direction = -1#終了条件の方向を割り当て。返り値が負の数になったら終了


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
"""
計算ベクトルxは以下の定義
x = [q0, q1, q2, q3, px, py, pz, rx, ry, rz, vx, vy, vz, ax, ay, az, time]
[0:4]は四元数
[4:7]は運動量ベクトル
[7:10]は位置ベクトル
[10:13]は角速度ベクトル
[13]はロール角
[14:17]はオイラー角
必要以上に計算をしているかもしれない。計算コストを下げるため後に改良
"""

X0_phase_1[0:4] = cos((LAUNCHER_ANGLE - np.pi / 2) / 2), 0, sin((LAUNCHER_ANGLE - np.pi / 2) / 2), 0#クオータニオン
X0_phase_1[4:7] = 0, 0, 0#運動量
X0_phase_1[7:10] = (Cg_BEFORE + GAP) * sin(LAUNCHER_ANGLE), 0, (Cg_BEFORE + GAP) * cos(LAUNCHER_ANGLE)#位置
X0_phase_1[10:13] = 0, 0, 0#角速度
X0_phase_1[13] = 0#ロール
X0_phase_1[14:17] = 0, -(90 - Constants.launcher()['Angle']), 0#オイラー角

#ランチャーレール上の微分方程式などを定義
def phase_1(t, X, wind_std, wind_angle):
    #変数の定義
    q0, q1, q2, q3 = X[0:4]
    px_launcher, py_launcher, pz_launcher = X[4:7]
    rx_launcher, ry_launcher, rz_launcher = X[7:10]
    ang_vx_align, ang_vy_align, ang_vz_align = X[10:13]
    roll_align = X[13]
    phi, theta, psi = X[14:17]

    
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

    diff_X = np.zeros_like(X)

    diff_X[0:4] = diff_q.components
    diff_X[4:7] = diff_p_launcher
    diff_X[7:10] = diff_r_launcher
    diff_X[10:13] = diff_ang_v_align
    diff_X[13] = diff_roll
    diff_X[14:17] = diff_phi, diff_theta, diff_psi

    return diff_X

#ランチャーレール外での微分方程式を定義
def phase_2(t, X, wind_std, wind_angle):

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

#パラシュート解散後の微分方程式を定義
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


OPEN_PARACHUTE = Constants.simulator()['Open_Parachute']


#以上の3phaseの微分方程式を解く
def solve_all(wind_std, wind_angle):

    if OPEN_PARACHUTE == 0:

        solve_phase_1 = solve(phase_1, wind_std, wind_angle, X0_phase_1, 0, launch_clear)

        t_start_phase_2 = solve_phase_1['t_final']
        X0_phase_2 = solve_phase_1['X_final']

        solve_phase_2 = solve(phase_2, wind_std, wind_angle, X0_phase_2, t_start_phase_2, landing)


        t1, t2 = solve_phase_1['t'], solve_phase_2['t'][1:]
        Xt1, Xt2 = solve_phase_1['Xt'], solve_phase_2['Xt'][:, 1:]

        t = np.concatenate([t1, t2], axis=0)
        Xt = np.concatenate([Xt1, Xt2], axis=1)

        q = qtn.as_quat_array(np.array([Xt[0], Xt[1], Xt[2], Xt[3]]).T)


        r_launcher = np.array([Xt[7], Xt[8], Xt[9]]).T
        r_gnd = Trans.launcher_to_gnd(r_launcher)
        down_range = np.array([np.sqrt(r_launcher[i, 1]**2 + r_launcher[i, 2]**2) for i in range(len(r_launcher)) ])
        altitude = r_launcher[:, 0]

        v_launcher = np.array([Xt[4] / CV.mass(t), Xt[5] / CV.mass(t), Xt[6] / CV.mass(t)]).T
        v_align = Trans.launcher_to_align(q, v_launcher)

        wind_launcher = Vectors.wind_launcher(altitude, wind_std, wind_angle)
        wind_align = Trans.launcher_to_align(q, wind_launcher)

        airflow_launcher = wind_launcher - v_launcher
        airflow_align = wind_align - v_align
        airflow_norm = norm(airflow_launcher, axis=1)

        Fn_align = Vectors.Fn_align(altitude, airflow_align)
        Ft_align = Vectors.Ft_align(altitude, airflow_align)

        g = Constants.physics()['g']
        a_launcher = (Vectors.gravity_launcher(t) + Trans.align_to_launcher(q, Fn_align + Ft_align)) / np.array([CV.mass(t) * g, CV.mass(t) * g, CV.mass(t) * g]).T
        a_norm = norm(a_launcher, axis=1)

        phase = np.concatenate([np.full_like(t1, 1), np.full_like(t2, 2)], axis=0)

        alpha_airflow = CV.alpha_airflow(airflow_align)
        alpha = CV.alpha(airflow_align)
        beta = CV.beta(airflow_align)

        air_density = CV.air_density(altitude)
        dp = CV.dp_as_kPa(altitude, airflow_align)

        index_launch_clear = len(t1) - 1
        index_apogee = np.argmax(altitude)
        index_Max_Q = np.argmax(dp)
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


    if OPEN_PARACHUTE == 1:

        solve_phase_1 = solve(phase_1, wind_std, wind_angle, X0_phase_1, 0, launch_clear)

        t_start_phase_2 = solve_phase_1['t_final']
        X0_phase_2 = solve_phase_1['X_final']

        solve_phase_2 = solve(phase_2, wind_std, wind_angle, X0_phase_2, t_start_phase_2, apogee)

        t_start_phase_3 = solve_phase_2['t_final']
        X0_phase_3 = solve_phase_2['X_final']

        solve_phase_3 = solve(phase_3, wind_std, wind_angle, X0_phase_3, t_start_phase_3, landing)


        t1, t2, t3 = solve_phase_1['t'], solve_phase_2['t'][1:], solve_phase_3['t'][1:]
        Xt1, Xt2, Xt3 = solve_phase_1['Xt'], solve_phase_2['Xt'][:, 1:], solve_phase_3['Xt'][:, 1:]

        t = np.concatenate([t1, t2, t3], axis=0)
        Xt = np.concatenate([Xt1, Xt2, Xt3], axis=1)

        t12= np.concatenate([t1, t2], axis=0)
        Xt12 = np.concatenate([Xt1, Xt2], axis=1)

        q12 = qtn.as_quat_array(np.array([Xt12[0], Xt12[1], Xt12[2], Xt12[3]]).T)

        r_launcher12 = np.array([Xt12[7], Xt12[8], Xt12[9]]).T
        r_launcher3 = np.array([Xt3[7], Xt3[8], Xt3[9]]).T
        r_launcher = np.concatenate([r_launcher12, r_launcher3], axis=0)


        r_gnd = Trans.launcher_to_gnd(r_launcher)
        down_range = np.array([np.sqrt(r_launcher[i, 1]**2 + r_launcher[i, 2]**2) for i in range(len(r_launcher)) ])

        altitude12 = r_launcher12[:, 0]
        altitude3 = r_launcher3[:, 0]
        altitude = r_launcher[:, 0]

        v_launcher = np.array([Xt[4] / CV.mass(t), Xt[5] / CV.mass(t), Xt[6] / CV.mass(t)]).T
        v_launcher12 = np.array([Xt12[4] / CV.mass(t12), Xt12[5] / CV.mass(t12), Xt12[6] / CV.mass(t12)]).T
        v_launcher3 = np.array([Xt3[4] / CV.mass(t3), Xt3[5] / CV.mass(t3), Xt3[6] / CV.mass(t3)]).T

        wind_launcher = Vectors.wind_launcher(altitude, wind_std, wind_angle)
        wind_launcher12 = Vectors.wind_launcher(altitude12, wind_std, wind_angle)
        wind_launcher3 = Vectors.wind_launcher(altitude3, wind_std, wind_angle)

        v_align12 = Trans.launcher_to_align(q12, v_launcher12)
        v_align3 = Trans.parachute_launcher_to_align(v_launcher3, wind_launcher3, v_launcher3)
        v_align = np.concatenate([v_align12, v_align3], axis=0)

        wind_align12 = Trans.launcher_to_align(q12, wind_launcher12)
        wind_align3 = Trans.parachute_launcher_to_align(v_launcher3, wind_launcher3, wind_launcher3)
        wind_align = np.concatenate([wind_align12, wind_align3], axis=0)

        airflow_launcher = wind_launcher - v_launcher
        airflow_align = wind_align - v_align
        airflow_align12 = wind_align12 - v_align12
        airflow_align3 = wind_align3 - v_align3
        airflow_norm = norm(airflow_launcher, axis=1)

        Fn_align = Vectors.Fn_align(altitude, airflow_align)
        Fn_align12 = Vectors.Fn_align(altitude12, airflow_align12)
        Fn_align3 = Vectors.Fn_align(altitude3, airflow_align3)

        Ft_align = Vectors.Ft_align(altitude, airflow_align)
        Ft_align12 = Vectors.Ft_align(altitude12, airflow_align12)
        Ft_align3 = Vectors.Ft_align(altitude3, airflow_align3)

        g = Constants.physics()['g']
        a_launcher12 = (Vectors.gravity_launcher(t12) + Trans.align_to_launcher(q12, Fn_align12 + Ft_align12)) / np.array([CV.mass(t12) * g, CV.mass(t12) * g, CV.mass(t12) * g]).T
        a_launcher3 = (Vectors.gravity_launcher(t3) + Trans.parachute_align_to_launcher(v_launcher3, wind_launcher3, Fn_align3 + Ft_align3)) / np.array([CV.mass(t3) * g, CV.mass(t3) * g, CV.mass(t3) * g]).T
        a_launcher = np.concatenate([a_launcher12, a_launcher3], axis=0)
        a_norm = norm(a_launcher, axis=1)

        phase = np.concatenate([np.full_like(t1, 1), np.full_like(t2, 2), np.full_like(t3, 3)], axis=0)

        alpha_airflow = CV.alpha_airflow(airflow_align)
        alpha = CV.alpha(airflow_align)
        beta = CV.beta(airflow_align)


        air_density = CV.air_density(altitude)
        dp = CV.dp_as_kPa(altitude, airflow_align)

        index_launch_clear = len(t1) - 1
        index_apogee = np.argmax(altitude)
        index_Max_Q = np.argmax(dp)
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

    return {'Basic' :
                {'t' : t, 'Xt' : Xt},

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



'''
for (wind_std, wind_angle) in product(range (0, 7, 1), np.linspace(0, 360, 9)[:-1]):
    pprint(solve_all(wind_std, wind_angle)['Summary'])
'''

pprint(solve_all(3, 0)['Summary'])

#print([solve_all(wind_std, wind_angle)['Summary']['Landing']['longitude'] for (wind_std, wind_angle) in product(range (0, 2, 1), np.linspace(0, 360, 3)[:-1])])



'''
if __name__ == '__main__':

    sq_t = solve_all(4, 0)['Basic']['t']
    sq_Xt = solve_all(4, 0)['Summary']['Air']

    vx_launcher = sq_Xt['dp']
    rz_launcher = sq_Xt['rho']


    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(sq_t, vx_launcher, 'r', label='vz_launcher [m/s]')

    ax1.set_xlabel('rz_launcher [m]')

    handler1, label1 = ax1.get_legend_handles_labels()
    ax1.legend(handler1, label1, loc='upper center', borderaxespad=-3.5)

    ax1.grid()
    plt.show()
'''




