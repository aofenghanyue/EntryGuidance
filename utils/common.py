import numpy as np
from numpy import sin, cos, tan, arccos
from scipy.optimize import brenth

class CoordinateTransformation:
    def __init__(self):
        # 虽然略有重复，但可以自定义mode名字
        self.mode_list = {
            "0": self._llcoords2GER
        }
        self._transform_list = [self._llcoords2GER]

    def coordinate_transformation(self, mode: str or int, **kwargs):
        """
        坐标转换函数
        :param mode:
            0: 经纬度转地球固连坐标单位向量
        :param kwargs:
            0:  lamb: 经度
                phi: 纬度
        :return:
        """
        if mode in self.mode_list.keys():
            return self.mode_list[mode](**kwargs)
        else:
            return self._transform_list[mode](**kwargs)

    def _llcoords2GER(self, lamb, phi):
        return np.array([cos(lamb) * cos(phi),
                         sin(lamb) * cos(phi),
                         sin(phi)])


coords_trans = CoordinateTransformation()


def heading_angle(lamb1, phi1, lamb2, phi2):
    """
    从lambda1,phi1位置到lambda2,phi2位置的大圆弧在两点处与经线的夹角
    :return:
    """
    # 大圆弧法向
    N = np.cross(coords_trans.coordinate_transformation(mode=0, lamb=lamb1, phi=phi1),
                 coords_trans.coordinate_transformation(mode=0, lamb=lamb2, phi=phi2))
    N = N / np.linalg.norm(N)
    # 两点的经线法向
    N1 = coords_trans.coordinate_transformation(mode=0, lamb=lamb1 + np.deg2rad(90), phi=0)
    N2 = coords_trans.coordinate_transformation(mode=0, lamb=lamb2 + np.deg2rad(90), phi=0)
    # 如果大圆弧法向z>0则夹角为180°-法向夹角
    if N[2] >= 0:
        res = np.deg2rad(180) - np.array([arccos(N.dot(N1)), arccos(N.dot(N2))])
    else:
        res = -np.deg2rad(180) + np.array([arccos(N.dot(N1)), arccos(N.dot(N2))])
    return res


def limit_num(num, abs_limit=np.inf, interval_limit=[-np.inf, np.inf], mode='abs'):
    """
    限制数字大小
    :param num: 要限制的数字
    :param abs_limit: 采用绝对值限制，数字绝对值超出该值会取边界
    :param interval_limit: 采用区间限制
    :param mode: 限制模式'abs'|'interval'
    :return:
    """
    sgn = np.sign(num)
    if mode == 'abs':
        if np.abs(num) > np.abs(abs_limit):
            return sgn * np.abs(abs_limit)
    elif mode == 'interval':
        if num > interval_limit[-1]:
            return interval_limit[-1]
        elif num < interval_limit[0]:
            return interval_limit[0]
    return num

def temp_f(alpha_temp, Ma, cl2cd, aero):
    """
    反解攻角的临时函数
    :param alpha_temp:
    :param cl2cd:
    :return:
    """
    CL, CD = aero.CLCD(Ma, alpha_temp)
    return CL/CD - cl2cd

def inv_alpha(Ma, CL2CD, aero):
    """
    由升阻比反解攻角
    :param Ma:
    :param CL2CD:
    :return:
    """
    # 估算最大升阻比对应的攻角
    alpha_max = 12 + 1.6*(Ma-4)/6
    try:
        r = brenth(temp_f, 0, np.deg2rad(alpha_max), args=(Ma, CL2CD, aero))
    except Exception as e:
        r = np.deg2rad(alpha_max)
    return r

if __name__ == '__main__':
    lamb1 = np.deg2rad(130)
    phi1 = np.deg2rad(5)
    lamb2 = np.deg2rad(130)
    phi2 = np.deg2rad(0)
    res = heading_angle(lamb1, phi1, lamb2, phi2)
    breakpoint()
