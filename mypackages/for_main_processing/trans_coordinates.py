
# mypy: ignore-errors
import numpy as np
from numpy import  deg2rad, sin, cos
from numpy.linalg import norm
import quaternion as qtn


from pathlib import Path

if __name__ == '__main__':
    import sys
    sys.path.append(str(Path(__file__).parent.parent))
    from for_pre_processing.input_from_Setting import Constants

else:
    from for_pre_processing.input_from_Setting import Constants


def rotation_matrix(quaternion):

    return qtn.as_rotation_matrix(quaternion)

def diff_quaternion(quaternion, ang_velocity):

    q_ang_velocity = qtn.from_vector_part(ang_velocity)

    diff_quaternion = (1/2) * quaternion * q_ang_velocity

    return diff_quaternion

def align_to_launcher(quaternion, vector_align):

    mat = rotation_matrix(quaternion)

    if isinstance(quaternion, qtn.quaternion):
        if vector_align.ndim > 1:
            vector_launcher = np.array([mat @ vec for vec in vector_align])

        else:
            vector_launcher = mat @ vector_align

    else:

        if vector_align.ndim > 1:
            vector_launcher = np.array([mat[i] @ vector_align[i] for i in range(len(vector_align))])

    return vector_launcher

def launcher_to_align(quaternion, vector_launcher):

    mat = rotation_matrix(quaternion.conj())

    if isinstance(quaternion, qtn.quaternion):
        if vector_launcher.ndim > 1:
            vector_align = np.array([mat @ vec for vec in vector_launcher])

        else:
            vector_align = mat @ vector_launcher

    else:

        if vector_launcher.ndim > 1:
            vector_align = np.array([mat[i] @ vector_launcher[i] for i in range(len(vector_launcher))])

    return vector_align


def parachute_align_to_launcher(velocity_Cg_launcher, wind_launcher, vector_align):

    airflow_launcher = wind_launcher - velocity_Cg_launcher

    if airflow_launcher.ndim > 1:

        vx = airflow_launcher[:, 0]
        vy = airflow_launcher[:, 1]
        vz = airflow_launcher[:, 2]
        v = np.array([norm(airflow_launcher[i]) if np.dot(velocity_Cg_launcher[i], wind_launcher[i]) < norm(velocity_Cg_launcher[i])**2 else -norm(airflow_launcher[i]) for i in range(len(airflow_launcher))]) + 1e-8

        mat = np.array([-(1/v[i]) * np.array([[vx[i], -vy[i], -vz[i]],
                                            [vy[i], -v[i] + vy[i]**2 / (v[i] - vx[i]), vy[i] * vz[i] / (v[i] - vx[i])],
                                            [vz[i], vy[i] * vz[i] / (v[i] - vx[i]), -v[i] + vz[i]**2 / (v[i] - vx[i])]]) for i in range(len(v))])


        vector_launcher = np.array([mat[i] @ vector_align[i] for i in range(len(v))])

    else:
        vx = airflow_launcher[0]
        vy = airflow_launcher[1]
        vz = airflow_launcher[2]

        if np.dot(velocity_Cg_launcher, wind_launcher) < norm(velocity_Cg_launcher)**2:
            v = norm(airflow_launcher) + 1e-8

        else:
            v = -norm(airflow_launcher) + 1e-8

        mat = np.array(-(1/v) * np.array([[vx, -vy, -vz],
                                        [vy, -v + vy**2 / (v - vx), vy * vz / (v - vx)],
                                        [vz, vy * vz / (v - vx), -v + vz**2 / (v - vx)]]))

        if vector_align.ndim > 1:
            vector_launcher = np.array([mat @ vec for vec in vector_align])

        else:
            vector_launcher = mat @ vector_align

    return vector_launcher

def parachute_launcher_to_align(velocity_Cg_launcher, wind_launcher, vector_launcher):

    airflow_launcher = wind_launcher - velocity_Cg_launcher

    if airflow_launcher.ndim > 1:

        vx = airflow_launcher[:, 0]
        vy = airflow_launcher[:, 1]
        vz = airflow_launcher[:, 2]
        v = np.array([norm(airflow_launcher[i]) if np.dot(velocity_Cg_launcher[i], wind_launcher[i]) < norm(velocity_Cg_launcher[i])**2 else -norm(airflow_launcher[i]) for i in range(len(airflow_launcher))]) + 1e-8


        mat = np.array([-(1/v[i]) * np.array([[vx[i], vy[i], vz[i]],
                                            [-vy[i], -v[i] + vy[i]**2 / (v[i] - vx[i]), vy[i] * vz[i] / (v[i] - vx[i])],
                                            [-vz[i], vy[i] * vz[i] / (v[i] - vx[i]), -v[i] + vz[i]**2 / (v[i] - vx[i])]]) for i in range(len(v))])

        vector_align = np.array([mat[i] @ vector_launcher[i] for i in range(len(v))])

    else:
        vx = airflow_launcher[0]
        vy = airflow_launcher[1]
        vz = airflow_launcher[2]

        if np.dot(velocity_Cg_launcher, wind_launcher) < norm(velocity_Cg_launcher)**2:
            v = norm(airflow_launcher) + 1e-8

        else:
            v = -norm(airflow_launcher) + 1e-8

        mat = np.array(-(1/v) * np.array([[vx, vy, vz],
                                        [-vy, -v + vy**2 / (v - vx), vy * vz / (v - vx)],
                                        [-vz, vy * vz / (v - vx), -v + vz**2 / (v - vx)]]))

        if vector_launcher.ndim > 1:
            vector_align = np.array([mat @ vec for vec in vector_launcher])

        else:
            vector_align = mat @ vector_launcher

    return vector_align


LAUNCHER_AZIMUTH = Constants.launcher()['Azimuth']

def launcher_to_gnd(vector_launcher):

    launcher_azimuth = deg2rad(LAUNCHER_AZIMUTH)
    mat = np.array([[0, -sin(launcher_azimuth), cos(launcher_azimuth)],
                    [0, cos(launcher_azimuth), sin(launcher_azimuth)],
                    [-1, 0, 0]])

    if vector_launcher.ndim > 1:
        vector_gnd = np.array([mat @ vec for vec in vector_launcher])

    else:
        vector_gnd = mat @ vector_launcher

    return vector_gnd

def gnd_to_launcher(vector_gnd):

    launcher_azimuth = deg2rad(LAUNCHER_AZIMUTH)
    mat = np.array([[0, 0, -1],
                    [-sin(launcher_azimuth), cos(launcher_azimuth), 0],
                    [cos(launcher_azimuth), sin(launcher_azimuth), 0]])

    if vector_gnd.ndim > 1:
        vector_launcher = np.array([mat @ vec for vec in vector_gnd])

    else:
        vector_launcher = mat @ vector_gnd

    return vector_launcher

def parallel_shift(vector_ref, vector):

    return vector - vector_ref


GEO_a = Constants.map()['Geodetic_System']['a']
GEO_F = Constants.map()['Geodetic_System']['F']
GEO_m0 = Constants.map()['Geodetic_System']['m0']

LAUNCHER_LON = Constants.launcher()['Longitude']
LAUNCHER_LAT = Constants.launcher()['Latitude']

LON0_DMS = Constants.map()['Geodetic_System']['Longitude']
LAT0_DMS = Constants.map()['Geodetic_System']['Latitude']

LON0 = sum(list(60**(-i) * float(LON0_DMS.split("'")[i]) for i in range(len(LON0_DMS.split("'")))))
LAT0 = sum(list(60**(-i) * float(LAT0_DMS.split("'")[i]) for i in range(len(LAT0_DMS.split("'")))))


def calc_lat_lon(x, y):
    """ 平面直角座標を緯度経度に変換する
    - input:
        (x, y): 変換したいx, y座標[m]
        (LAT0, LON0): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
    - output:
        latitude:  緯度[度]
        longitude: 経度[度]
    """
    # 平面直角座標系原点をラジアンに直す
    phi0_rad = deg2rad(LAT0)
    lambda0_rad = deg2rad(LON0)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n**2)/4. + (n**4)/64.
        A1 = -     (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
        A2 =     (15./16)*( n**2 - (n**4)/4. )
        A3 = -   (35./48)*( n**3 - (5./16)*(n**5) )
        A4 =   (315./512)*( n**4 )
        A5 = -(693./1280)*( n**5 )
        return np.array([A0, A1, A2, A3, A4, A5])

    def beta_array(n):
        b0 = np.nan # dummy
        b1 = (1./2)*n - (2./3)*(n**2) + (37./96)*(n**3) - (1./360)*(n**4) - (81./512)*(n**5)
        b2 = (1./48)*(n**2) + (1./15)*(n**3) - (437./1440)*(n**4) + (46./105)*(n**5)
        b3 = (17./480)*(n**3) - (37./840)*(n**4) - (209./4480)*(n**5)
        b4 = (4397./161280)*(n**4) - (11./504)*(n**5)
        b5 = (4583./161280)*(n**5)
        return np.array([b0, b1, b2, b3, b4, b5])

    def delta_array(n):
        d0 = np.nan # dummy
        d1 = 2.*n - (2./3)*(n**2) - 2.*(n**3) + (116./45)*(n**4) + (26./45)*(n**5) - (2854./675)*(n**6)
        d2 = (7./3)*(n**2) - (8./5)*(n**3) - (227./45)*(n**4) + (2704./315)*(n**5) + (2323./945)*(n**6)
        d3 = (56./15)*(n**3) - (136./35)*(n**4) - (1262./105)*(n**5) + (73814./2835)*(n**6)
        d4 = (4279./630)*(n**4) - (332./35)*(n**5) - (399572./14175)*(n**6)
        d5 = (4174./315)*(n**5) - (144838./6237)*(n**6)
        d6 = (601676./22275)*(n**6)
        return np.array([d0, d1, d2, d3, d4, d5, d6])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = GEO_m0
    a = GEO_a
    F = GEO_F

    # (1) n, A_i, beta_i, delta_iの計算
    n = 1. / (2*F - 1)
    A_array = A_array(n)
    beta_array = beta_array(n)
    delta_array = delta_array(n)

    # (2), S, Aの計算
    A_ = ( (m0*a)/(1.+n) )*A_array[0]
    S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) )

    # (3) xi, etaの計算
    xi = (x + S_) / A_
    eta = y / A_

    # (4) xi', eta'の計算
    xi2 = xi - np.sum(np.multiply(beta_array[1:],
                                np.multiply(np.sin(2*xi*np.arange(1,6)),
                                            np.cosh(2*eta*np.arange(1,6)))))
    eta2 = eta - np.sum(np.multiply(beta_array[1:],
                                np.multiply(np.cos(2*xi*np.arange(1,6)),
                                            np.sinh(2*eta*np.arange(1,6)))))

    # (5) chiの計算
    chi = np.arcsin( np.sin(xi2)/np.cosh(eta2) ) # [rad]
    latitude = chi + np.dot(delta_array[1:], np.sin(2*chi*np.arange(1, 7))) # [rad]

    # (6) 緯度(latitude), 経度(longitude)の計算
    longitude = lambda0_rad + np.arctan( np.sinh(eta2)/np.cos(xi2) ) # [rad]

    # ラジアンを度になおしてreturn
    return np.rad2deg(latitude), np.rad2deg(longitude) # [deg]


def calc_x_y(latitude, longitude):
    """ 緯度経度を平面直角座標に変換する
    - input:
        (phi_deg, lambda_deg): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
        (phi0_deg, lambda0_deg): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
    - output:
        x: 変換後の平面直角座標[m]
        y: 変換後の平面直角座標[m]
    """
    # 緯度経度・平面直角座標系原点をラジアンに直す
    phi_rad = deg2rad(latitude)
    lambda_rad = deg2rad(longitude)
    phi0_rad = deg2rad(LAT0)
    lambda0_rad = deg2rad(LON0)

    # 補助関数
    def A_array(n):
        A0 = 1 + (n**2)/4. + (n**4)/64.
        A1 = -     (3./2)*( n - (n**3)/8. - (n**5)/64. ) 
        A2 =     (15./16)*( n**2 - (n**4)/4. )
        A3 = -   (35./48)*( n**3 - (5./16)*(n**5) )
        A4 =   (315./512)*( n**4 )
        A5 = -(693./1280)*( n**5 )
        return np.array([A0, A1, A2, A3, A4, A5])

    def alpha_array(n):
        a0 = np.nan # dummy
        a1 = (1./2)*n - (2./3)*(n**2) + (5./16)*(n**3) + (41./180)*(n**4) - (127./288)*(n**5)
        a2 = (13./48)*(n**2) - (3./5)*(n**3) + (557./1440)*(n**4) + (281./630)*(n**5)
        a3 = (61./240)*(n**3) - (103./140)*(n**4) + (15061./26880)*(n**5)
        a4 = (49561./161280)*(n**4) - (179./168)*(n**5)
        a5 = (34729./80640)*(n**5)
        return np.array([a0, a1, a2, a3, a4, a5])

    # 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
    m0 = GEO_m0
    a = GEO_a
    F = GEO_F

    # (1) n, A_i, alpha_iの計算
    n = 1. / (2*F - 1)
    A_array = A_array(n)
    alpha_array = alpha_array(n)

    # (2), S, Aの計算
    A_ = ( (m0*a)/(1.+n) )*A_array[0] # [m]
    S_ = ( (m0*a)/(1.+n) )*( A_array[0]*phi0_rad + np.dot(A_array[1:], np.sin(2*phi0_rad*np.arange(1,6))) ) # [m]

    # (3) lambda_c, lambda_sの計算
    lambda_c = np.cos(lambda_rad - lambda0_rad)
    lambda_s = np.sin(lambda_rad - lambda0_rad)

    # (4) t, t_の計算
    t = np.sinh( np.arctanh(np.sin(phi_rad)) - ((2*np.sqrt(n)) / (1+n))*np.arctanh(((2*np.sqrt(n)) / (1+n)) * np.sin(phi_rad)) )
    t_ = np.sqrt(1 + t*t)

    # (5) xi', eta'の計算
    xi2  = np.arctan(t / lambda_c) # [rad]
    eta2 = np.arctanh(lambda_s / t_)

    # (6) x, yの計算
    x = A_ * (xi2 + np.sum(np.multiply(alpha_array[1:],
                                    np.multiply(np.sin(2*xi2*np.arange(1,6)),
                                                np.cosh(2*eta2*np.arange(1,6)))))) - S_ # [m]
    y = A_ * (eta2 + np.sum(np.multiply(alpha_array[1:],
                                        np.multiply(np.cos(2*xi2*np.arange(1,6)),
                                                    np.sinh(2*eta2*np.arange(1,6)))))) # [m]
    # return
    return x, y # [m]

def gnd_to_latlon(xy_gnd):

    xy_launcher = np.array(calc_x_y(LAUNCHER_LAT, LAUNCHER_LON))
    xy_RC = xy_gnd + xy_launcher

    if xy_gnd.ndim > 1:
        latlon = np.array([np.array(calc_lat_lon(xy_RC[i, 0], xy_RC[i, 1])) for i in range(len(xy_gnd))])

    else:
        latlon = np.array(calc_lat_lon(xy_RC[0], xy_RC[1]))

    return latlon










if __name__ == '__main__':
    q1 = np.quaternion(1, 1, 0, 0)
    q2 = np.quaternion(1, 0, 1, 0)
    qs = np.array([q1, q2])

    q0 = np.quaternion(0, 1, 0, 0)


    vec1 = np.array([1, 0, 0])
    vec2 = np.array([0, 1, 0])
    vec3 = np.array([0, 0, 1])
    vec4 = np.array([-1, 0, 0])
    vec5 = np.array([0, 0, 0])

    vecs = np.array([vec1, vec2])
    vecs2 = np.array([vec1, vec1])
    vecs3 = np.array([vec2, vec2])



