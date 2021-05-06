import numpy as np


class Aerodynamic:
    def __init__(self):
        pass

    def CL(self, ma, alpha):
        pass

    def CD(self, ma, alpha):
        pass

    def CLCD(self, ma, alpha):
        pass


class AerodynamicCAVH(Aerodynamic):
    def __init__(self):
        self.CL_alpha_coef = np.array([2.17569024780558e-05, -0.00144978843672489, 0.0646458686885526]) * 180 / np.pi
        self.CL0_coef = np.array(
            [[-3.42376047505561e-07, 1.17150762343450e-05, -0.000100700419002099, 0.000402323033094062,
              -0.00157783752335338, -0.150000000000000],
             [0, 0, 0, 0, 0.00599999999999959, -0.203333333333331],
             [0, -1.07407407407322e-05, 0.00106444444444371, -0.0393129629629393, 0.641782222221885, -3.99249629629450]]
        )
        self.K_coef = np.array(
            [[-2.46305771748035e-07, 6.72940229873472e-06, -3.77797321542415e-05, -0.000252392205651594,
              0.00182522206336354, 0.00372306174504015, 0.283000000000000],
             [0, 1.69353854878041e-07, -1.66964107194587e-05, 0.000621031997336427, -0.0109940650955127,
              0.100752055995445, -0.0849464753271712]])
        self.CD0_coef = np.array(
            [[-0.259259259259259, 0.471759259259259, -0.102500000000000, 0, 0.0500000000000000],
             [0, 0.00258220855317959, -0.0232695113776603, 0.0446916863566490, 0.155416016375958],
             [9.25268743745020e-05, -0.00257436590032047, 0.0266344981080193, -0.131000627832209, 0.366218315445375],
             [6.05023225121903e-07, -3.43552643423171e-05, 0.000703290091255527, -0.00669225369285760,
              0.0954632512438941]])

    def CL(self, ma, alpha):
        """
        升力系数
        :param ma: 马赫数
        :param alpha: 攻角 rad
        :return:
        """
        CL_alpha = self.CL_alpha_coef.dot([ma ** 2, ma, 1])
        if ma <= 10:
            row = 0
        elif ma <= 19:
            row = 1
        else:
            row = 2
        CL0 = self.CL0_coef[row, :].dot([ma ** i for i in range(6)][::-1])

        return CL0 + CL_alpha * alpha

    def CLCD(self, ma, alpha):
        CL = self.CL(ma, alpha)
        if ma <= 12:
            row = 0
        else:
            row = 1
        k = self.K_coef[row, :].dot([ma ** i for i in range(7)][::-1])

        if ma <= 1.2:
            row = 0
        elif ma <= 3.5:
            row = 1
        elif ma <= 10:
            row = 2
        else:
            row = 3
        CD0 = self.CD0_coef[row, :].dot([ma ** i for i in range(5)][::-1])
        CD = CD0 + k * CL ** 2
        return CL, CD

    def CD(self, ma, alpha):
        _, CD = self.CLCD(ma, alpha)
        return CD


def CL(aero_model: Aerodynamic, ma, alpha):
    return aero_model.CL(ma, alpha)


def CD(aero_model: Aerodynamic, ma, alpha):
    return aero_model.CD(ma, alpha)


def CLCD(aero_model: Aerodynamic, ma, alpha):
    return aero_model.CLCD(ma, alpha)


if __name__ == '__main__':
    aero = AerodynamicCAVH()
    print(aero.CL(10, 10 / 57.3))
