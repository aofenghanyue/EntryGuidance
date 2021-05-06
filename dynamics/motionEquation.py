# -*- coding: utf-8 -*-
import abc
import numpy as np
from numpy import sin, cos, tan
from database.Constant import earth as e


class MotionEquation:
    def __init__(self):
        super(MotionEquation, self).__init__()

    @abc.abstractmethod
    def equation(self, x, y, meta={}):
        pass


class ME6D(MotionEquation):
    def equation(self, x, y, meta={}):
        """
        六自由度微分方程
        :param x: t
        :param y: array
        [lam, phi, H, V, gamma, psi]
        [经度，纬度，高度，速度，弹道倾角，弹道偏角]
        :param meta: {L, D, m, alpha, sigma}
        :return:
        """
        # 读取传入的参数
        lam, phi, H, V, gamma, psi = y[:]
        L, D, m, alpha, sigma = meta["L"], meta["D"], meta["m"], meta["attack_angle"], meta["bank_angle"]
        # 部分常量
        Re = e.Re
        we = e.omega_e
        g = e.g(H)

        dy = np.array([0] * len(y))
        dy[0] = V * cos(gamma) * sin(psi) / (Re + H) / cos(phi)
        dy[1] = V * cos(gamma) * cos(psi) / (Re + H)
        dy[2] = V * sin(gamma)
        dy[3] = -D / m - g * sin(gamma) + we ** 2 * (Re + H) * cos(phi) ** 2 * sin(gamma) - we ** 2 * (Re + H) * sin(
            phi) * cos(phi) * cos(gamma) * cos(psi)
        dy[4] = (L * cos(sigma) / m - g * cos(gamma) + V ** 2 * cos(gamma) / (Re + H) + we ** 2 * (Re + H) * cos(
            phi) ** 2 * cos(gamma) + 2 * V * we * cos(phi) * sin(psi) + we ** 2 * (Re + H) * sin(phi) * cos(phi) * sin(
            gamma) * cos(psi)) / V
        dy[5] = (L * sin(sigma) / m / cos(gamma) + V ** 2 * cos(gamma) * sin(psi) * tan(phi) / (Re + H) - we ** 2 * (
                Re + H) * sin(phi) * cos(phi) * sin(psi) / cos(gamma) + 2 * V * we * sin(phi) - 2 * V * we * cos(
            phi) * tan(gamma) * cos(psi)) / V

        return dy
