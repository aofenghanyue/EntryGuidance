from dynamics.aerodynamic import Aerodynamic
from dynamics.motionEquation import MotionEquation
from database.Atmosphere import atmosphereISA as ISA
from database.Constant import earth as e
from store.status import MissileStatus


class Missile:
    def __init__(self, motion_equation: MotionEquation = MotionEquation(), aerodynamic: Aerodynamic = Aerodynamic(),
                 params=None, status: MissileStatus = MissileStatus()):
        """
        导弹
        :param motion_equation: 运动/动力学方程
        :param aerodynamic: 气动参数CL,CD(alpha, ma)
        :param params: 导弹固有参数，如参考面积等，为对象（如CAVHParams()）
        :param status: 导弹状态集
        """
        self.equation = motion_equation
        self.aero = aerodynamic
        self.p = params

        # 导弹状态，需和微分方程对应，self.status
        self.status = status
        self.launched = False
        # 导弹本地时间，从发射计时 self.local_t
        self.local_t = 0
        # 导弹控制参数 self.control
        self.control: dict = {}
        # 导弹制导结果 self.guide
        self.guide: dict = {}

    @property
    def ma(self):
        """
        :return: 当前马赫
        """
        return self.status.velocity / ISA.a(self.status.height)

    def CLCD(self):
        CL, CD = self.aero.CLCD(self.ma, self.control["attack_angle"])
        q_inf = ISA.rho(self.status.height) * self.status.velocity ** 2 / 2 * self.p.reference_area
        return q_inf * CL, q_inf * CD

    @property
    def E(self):
        # 当前单位能量
        return self.status.velocity ** 2 / 2 - e.mu / (e.Re + self.status.height)
