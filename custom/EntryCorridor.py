# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
from scipy import sqrt
from scipy.optimize import fsolve

from database.vehicleParams import CAVHParams
from database.Atmosphere import atmosphereISA as ATM
from dynamics.aerodynamic import AerodynamicCAVH
from database.Constant import earth as e
from utils.interpolate import Interp1


class EntryCorridorCAV:
    def __init__(self, alpha_profile, params=CAVHParams(), aero=AerodynamicCAVH()):
        self.p = params
        self.aero = aero
        self.alpha = alpha_profile

        self.init_h = [5e4, 5e4, 5e4, 5e4]

        self.corridor_hv: pd.DataFrame = self.gen_border()
        self.corridor_he: pd.DataFrame = self.hv2he_border()

        self.L2D_E, self.interp_dCL_dE, self.interp_dCD_dE, self.interp_CL_E, self.interp_CD_E = self.dataframe2curve(
            self.corridor_he)
        self.dHmindE_E, self.interp_Hmin_E = self.dHmin_dE(self.corridor_he)

    def q(self, v, h):
        return ATM.rho(h) * v ** 2 / 2 - self.p.q_max

    def dot_Q(self, v, h):
        return self.p.k_Q * sqrt(ATM.rho(h)) * v ** 3.15 - self.p.dotQ_max

    def n(self, v, h):
        ma = v / ATM.a(h)
        E = e.E(v, h)
        CL = self.aero.CL(ma, self.alpha(E))
        return ATM.rho(h) * v ** 2 / 2 * self.p.reference_area * CL / self.p.m / e.g0 - self.p.n_max

    def glide(self, v, h):
        return v ** 2 / (e.Re + h) - e.g0 + (self.n(v, h) + self.p.n_max) * e.g0

    def gen_border(self):
        v_array = np.linspace(7100, 1900, 600)
        return pd.DataFrame([h for h in map(self.h_for_v, v_array)], columns=['v', 'q', 'dotQ', 'n', 'glide'])

    def hv2he_border(self):
        E_list = np.linspace(-3.71e7, -6.04e7, 5000)
        # df_he: 'v', 'q', 'dotQ', 'n', 'glide'
        df_he = pd.DataFrame()
        df_he['E'] = E_list

        for col in self.corridor_hv.columns[1:]:
            interp = Interp1(e.E(self.corridor_hv['v'], self.corridor_hv[col]), self.corridor_hv[col])
            df_he[col] = interp.pre(E_list)

        df_he['minH'] = df_he[['q', 'dotQ', 'n']].max(axis=1)
        df_he['midH'] = (df_he['minH'] + df_he['glide']) / 2
        return df_he

    def dataframe2curve(self, he):
        # 返回一个函数L2D_plan(E)
        h = he['midH']
        E = he['E']
        v = np.sqrt((E + e.mu / (e.Re + h)) * 2)
        L2D = []
        CL, CD = [], []
        for h_i, E_i, v_i in zip(h, E, v):
            ma = v_i / ATM.a(h_i)
            alpha = self.alpha(E_i)
            CL_num, CD_num = self.aero.CLCD(ma, alpha)
            CL.append(CL_num)
            CD.append(CD_num)
            L2D.append(CL_num / CD_num)
        interp_CL_E = Interp1(E, CL)
        interp_CD_E = Interp1(E, CD)
        interp_l2d_E = Interp1(E, np.array(L2D))
        temp3 = np.array(E[:-2]) - np.array(E[2:])
        temp1 = (np.array(CL[:-2]) - np.array(CL[2:])) / temp3
        interp_dCL_dE = Interp1(E[1:-1], temp1)
        temp2 = (np.array(CD[:-2]) - np.array(CD[2:])) / temp3
        interp_dCD_dE = Interp1(E[1:-1], temp2)
        return interp_l2d_E.pre, interp_dCL_dE.pre, interp_dCD_dE.pre, interp_CL_E.pre, interp_CD_E.pre

    def h_for_v(self, v):
        h_q = self.h_corridor(self.q, v, self.init_h[0])[0]
        h_dot_Q = self.h_corridor(self.dot_Q, v, self.init_h[1])[0]
        h_n = self.h_corridor(self.n, v, self.init_h[2])[0]
        h_glide = self.h_corridor(self.glide, v, self.init_h[3])[0]
        self.init_h = [h_q, h_dot_Q, h_n, h_glide]

        return [v, h_q, h_dot_Q, h_n, h_glide]

    def h_corridor(self, f, v, x0):
        return fsolve(lambda h: f(v, h), x0)

    def dHmin_dE(self, he):
        h = he['minH']
        E = he['E']
        temp = (np.array(h[:-2]) - np.array(h[2:])) / (np.array(E[:-2]) - np.array(E[2:]))
        interp = Interp1(E[1:-1], temp)
        interp2 = Interp1(E, h)
        return interp.pre, interp2.pre

    @property
    def average_h_e(self):
        return self.corridor_he["midH"].mean()


def alpha_pre_plan(E):
    E_alpha = -5.55e7
    # Vf = 2000 m/s; Hf = 25km
    E_final = 2000 ** 2 / 2 - 6.674e-11 * 5.9722e24 / (6.371e6 + 2.5e4)
    alpha1 = 10 * np.pi / 180
    alpha2 = 6 * np.pi / 180
    if E > E_alpha:
        return alpha1
    else:
        return ((E_alpha - E) / (E_alpha - E_final)) ** 2 * (alpha2 - alpha1) + alpha1


cav_corridor = EntryCorridorCAV(alpha_profile=alpha_pre_plan)
