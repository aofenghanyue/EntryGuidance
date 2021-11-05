from dynamics.motionEquation import MotionEquation


class Integral:
    def next_step(self, f, x, y0, h, meta={}, need_dy=False):
        """
        单步积分
        :param f: 被积微分方程
        :param x: 自变量
        :param y0: 状态初值
        :param h: 步长
        :param meta: 附加参数
        :param need_dy: 是否需要输出微分信息
        :return: 新状态
        """
        return y0


class RungeKutta4(Integral):
    # 四阶龙格库塔法
    def __init__(self):
        super(RungeKutta4, self).__init__()

    def next_step(self, f, x, y, h, meta={}, need_dy=False):
        """
        单步积分
        :parameter f: 被积微分方程组
        :parameter x: 自变量 num
        :parameter y: 积分初值 array
        :parameter meta: 传递积分中可能用到的参数 dict
        :param need_dy: 是否需要输出微分信息
        :param h: 积分步长
        :return: array
        """
        k1 = f(x, y, meta=meta)
        k2 = f(x + h / 2, y + h * k1 / 2, meta=meta)
        k3 = f(x + h / 2, y + h * k2 / 2, meta=meta)
        k4 = f(x + h, y + h * k3, meta=meta)
        y_next = y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        if need_dy:
            return y_next, k1
        else:
            return y_next, None
