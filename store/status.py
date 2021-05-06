import numpy as np


class MissileStatus:
    def __init__(self, independent_variable="t", integral_variable: list = []):
        """
        导弹状态
        :param independent_variable: 自变量，通常默认为时间t
        :param integral_variable: 积分变量，输入参数列表["v","h",...]，与微分方程的变量要对应
        """
        self.independent_key = independent_variable
        self.integral_key = integral_variable

        # 此处添加各种状态变量名，可以在某一仿真中不使用
        self.t = 0
        self.longitude = 0
        self.latitude = 0
        self.height = 0
        self.velocity = 0
        self.path_angle = 0
        self.heading_angle = 0

    def change_stat(self, status_dict: dict):
        # 传入参数：status_dict为包含要修改变量的字典
        for k in status_dict.keys():
            setattr(self, k, status_dict[k])

    @property
    def x(self):
        return getattr(self, self.independent_key)

    @property
    def y(self):
        return np.array([getattr(self, k) for k in self.integral_key])

    def status_dict(self):
        return {k: getattr(self, k) for k in self.integral_key + [self.independent_key]}


class ME6DStatus(MissileStatus):
    def __init__(self, independent_variable="t",
                 integral_variable=["longitude", "latitude", "height", "velocity", "path_angle", "heading_angle"]):
        super(ME6DStatus, self).__init__(independent_variable, integral_variable)
