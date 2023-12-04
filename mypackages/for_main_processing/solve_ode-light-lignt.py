# mypy: ignore-errors
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


TIME_STEP = Constants.simulator()['Time_Step']
TIME_MAX = Constants.simulator()['Time_Max']



def solve(ode, wind_std, wind_angle, X0, t_start, events):
    time_step = TIME_STEP
    t_max = TIME_MAX

    sol = solve_ivp(ode, (t_start, t_max), X0, args=(wind_std, wind_angle), dense_output=True, events=events, rtol = 1e-5)

    t_final = sol.t[-1]
    X_final = sol.y[:, -1]

    t = np.arange(t_start, t_final, time_step)
    Xt = sol.sol(t)

    return {'t_final' : t_final, 'X_final' : X_final, 't' : t, 'Xt' : Xt, 'fun_Xt' : sol.sol}


LAUNCHER_LENGTH = Constants.launcher()['Length']
Cg_BEFORE = Constants.rocket()['Cg']['Before']
GAP = Constants.rocket()['Gap_Tail_Launcher']

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

LAUNCHER_ANGLE = deg2rad(Constants.launcher()['Angle'])

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


        r_launcher = np.array([Xt[7], Xt[8], Xt[9]]).T
        r_gnd = Trans.launcher_to_gnd(r_launcher)
        down_range = np.array([np.sqrt(r_launcher[i, 1]**2 + r_launcher[i, 2]**2) for i in range(len(r_launcher)) ])

        index_landing = -1

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



        r_launcher = np.array([Xt[7], Xt[8], Xt[9]]).T
        r_gnd = Trans.launcher_to_gnd(r_launcher)
        down_range = np.array([np.sqrt(r_launcher[i, 1]**2 + r_launcher[i, 2]**2) for i in range(len(r_launcher)) ])

        index_landing = -1

        t_landing, down_range_landing = t[index_landing], down_range[index_landing]
        latitude_landing = Trans.gnd_to_latlon(np.array([r_gnd[index_landing, 0], r_gnd[index_landing, 1]]))[0]
        longitude_landing = Trans.gnd_to_latlon(np.array([r_gnd[index_landing, 0], r_gnd[index_landing, 1]]))[1]

    return {'Basic' :
                {'t' : t, 'Xt' : Xt},

            'Summary' :
                {'Wind' :
                    {'wind' : wind_std, 'angle' : wind_angle},

                'Landing' :
                    {'time' : t_landing, 'DownRange' : down_range_landing, 'longitude' : longitude_landing, 'latitude' : latitude_landing}}}



if Constants.simulator()['Wind_Model'] == 0:
    WIND_STD_INIT = Constants.wind()['Power_law']['Wind_STD_Init']
    INTERVAL_WIND_STD = Constants.wind()['Power_law']['Interval_Wind_STD']
    VARIATION_WIND_STD = int(Constants.wind()['Power_law']['Variation_Wind_STD'])

    WIND_AZIMUTH_INIT = Constants.wind()['Power_law']['Wind_Azimuth_Init']
    VARIATION_WIND_AZIMUTH = int(Constants.wind()['Power_law']['Variation_Wind_Azimuth'])

    print(np.arange (WIND_STD_INIT, WIND_STD_INIT + VARIATION_WIND_STD * INTERVAL_WIND_STD, INTERVAL_WIND_STD))
    print(np.linspace(WIND_AZIMUTH_INIT, 360, VARIATION_WIND_AZIMUTH + 1)[:-1])

    for (wind_std, wind_angle) in product(np.arange (WIND_STD_INIT, WIND_STD_INIT + VARIATION_WIND_STD * INTERVAL_WIND_STD, INTERVAL_WIND_STD), np.linspace(WIND_AZIMUTH_INIT, 360, VARIATION_WIND_AZIMUTH + 1)[:-1]):
        pprint(solve_all(wind_std, wind_angle)['Summary']['Landing']['latitude'])

    for (wind_std, wind_angle) in product(np.arange (WIND_STD_INIT, WIND_STD_INIT + VARIATION_WIND_STD * INTERVAL_WIND_STD, INTERVAL_WIND_STD), np.linspace(WIND_AZIMUTH_INIT, 360, VARIATION_WIND_AZIMUTH + 1)[:-1]):
        pprint(solve_all(wind_std, wind_angle)['Summary']['Landing']['longitude'])


if Constants.simulator()['Wind_Model'] == 1:
    pprint(solve_all(0, 0)['Summary']['Landing']['latitude'])
    pprint(solve_all(0, 0)['Summary']['Landing']['longitude'])




'''
if __name__ == '__main__':

    sq_t = solve_all(0, 180)['Basic']['t']
    sq_Xt = solve_all(0, 180)['Basic']['Xt']

    x_launcher = sq_Xt[7]
    z_launcher = sq_Xt[9]
    


    fig, ax1 = plt.subplots()


    ax1.plot(z_launcher, x_launcher, 'r', label='x_launcher [m]')

    ax1.set_xlabel('z_launcher [m]')

    handler1, label1 = ax1.get_legend_handles_labels()
    ax1.legend(handler1, label1, loc='upper center', borderaxespad=-3.5)

    ax1.grid()
    plt.show()
'''




