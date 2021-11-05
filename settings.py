import math
import numpy as np
from numpy import seterr

seterr(all='raise')

# 最大迭代次数
# MAXITER = math.inf
MAXITER = 100000

# 初始时间
t0 = 0

# 传入制导模块的参数
# self: 仿真对象本身
# t: 全局时间
# iter: 迭代次数
ParamsToGuide = ["t", "self"]

# 仿真案例设置
cases = [
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(15)},
     'TargetInitStatus': {"longitude": np.deg2rad(165),
                          "latitude": np.deg2rad(40)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(60)},
     'TargetInitStatus': {"longitude": np.deg2rad(120),
                          "latitude": np.deg2rad(0)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(120)},
     'TargetInitStatus': {"longitude": np.deg2rad(90),
                          "latitude": np.deg2rad(-40)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(180)},
     'TargetInitStatus': {"longitude": np.deg2rad(10),
                          "latitude": np.deg2rad(-50)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(-140)},
     'TargetInitStatus': {"longitude": np.deg2rad(-50),
                          "latitude": np.deg2rad(-40)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(-85)},
     'TargetInitStatus': {"longitude": np.deg2rad(-90),
                          "latitude": np.deg2rad(0)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },

    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(-10)},
     'TargetInitStatus': {"longitude": np.deg2rad(-165),
                          "latitude": np.deg2rad(40)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
{'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(0),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(89)},
     'TargetInitStatus': {"longitude": np.deg2rad(80),
                          "latitude": np.deg2rad(0)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4}
     },
]
# 测试通过0，1，2，3，4
#
case_num = 2

# 导弹初始化状态
MissileInitStatus = cases[case_num]['MissileInitStatus']
# 导弹终止状态
MissileEndStatus = cases[case_num]['MissileEndStatus']
# 目标初始状态
TargetInitStatus = cases[case_num]['TargetInitStatus']

# 制导相关参数

# 是否考虑最大倾侧角
CONSIDER_SIGMA_MAX = True
# K_GAMMA = 3  # 下降段攻角反馈系数
K_GAMMA_SGP = 3  # 滑翔段TDCT反馈系数
ERR_TOL = 3e-4  # 进入平稳滑翔段时弹道倾角与平稳滑翔弹道倾角的容忍误差
K_SIGMA = -50  # 最大倾侧角中的系数
K_ALPHA = 5 * np.pi / 1E7 / 1.8
# 要从制导参数直接导入到控制参数的值名称↓
CONTROL_PARAM_LIST = ["m", "attack_angle", "bank_angle"]
# 除控制参数和状态外要从制导模块保存的数据
GUIDE_SAVE_PARAM = [
    # "sigma_max_L1",
    # "sigma_max_Lmax",
    # "sigma_max_dHmindE",
    # "sigma_max_dHdE",
    "sigma_max",
    "gamma_sg",
    "s_go",
    "sgo_ref",
    "ref_psi",
    "delta_psi",
    'q'
    # "sigma_aap_aL2",
    # "sigma_aap_deltaL2m"
]
# 更新EBR2时最大迭代次数
MAX_EBR2_ITER = 10

# 保存数据
STORE_DATA = f'store/data_saved/simulation_case{case_num}.csv'
