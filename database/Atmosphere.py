# -*- coding: utf-8 -*-
import os
import numpy as np
from utils.interpolate import Interp1


class AtmosphereISA:
    def __init__(self):
        self.ISA = np.genfromtxt(os.path.dirname(__file__) + "/ISA.txt")
        self.T_h, self.rho_h, self.a_h = (Interp1(self.ISA[:, 0] * 1e3, self.ISA[:, i + 1]).pre for i in range(3))

    def T(self, h):
        return self.T_h(h)

    # def rho(self, h):
    #     return self.rho_h(h)
    def rho(self, h):
        return 1.225*np.exp(-h*0.00015)

    def a(self, h):
        return self.a_h(h)


atmosphereISA = AtmosphereISA()

if __name__ == '__main__':
    a = AtmosphereISA()
    print(a.rho(2.5e4))
