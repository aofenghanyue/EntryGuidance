import math

# 最大迭代次数
# MAXITER = math.inf
MAXITER = 10

# 初始时间
t0 = 0

# 传入制导模块的参数
# self: 仿真对象本身
# t: 全局时间
# iter: 迭代次数
ParamsToGuide = ["t"]

# 导弹初始化状态
MissileInitStatus = {"t": 0,
                     "longitude": 0,
                     "latitude": 0,
                     "height": 0,
                     "velocity": 0,
                     "path_angle": 0,
                     "heading_angle": 0}

# 目标初始状态
TargetInitStatus = {"longitude": 0,
                    "latitude": 0}
