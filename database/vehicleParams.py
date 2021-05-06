import numpy as np


class CAVHParams:
    def __init__(self):
        # 参考面积、质量：S 750 in^2; m 2000 Ibs
        self.reference_area = 0.4839  # m^2
        self.m = 907.185  # kg

        # 限制参数
        self.k_Q = 1.5e-8
        self.dotQ_max = 500  # W/cm^2
        self.q_max = 1.5e5  # Pa
        self.n_max = 2  # g

        # 飞控系统限制
        self.alpha_max = np.deg2rad(20)
        self.max_L2D_alpha = np.deg2rad(10)


CAVH = CAVHParams()
