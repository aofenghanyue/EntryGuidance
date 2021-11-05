import math
import numpy as np
from numpy import seterr

seterr(all='raise')

# 最大迭代次数
# MAXITER = math.inf
MAXITER = 2.6E6

# 初始时间
t0 = 0

# 最小仿真步长
MIN_H = 1E-2
# 各导弹仿真初始步长
INI_STEP = 0.1

# 传入制导模块的参数
# self: 仿真对象本身
# t: 全局时间
# iter: 迭代次数
ParamsToGuide = ["t", "min_h", "self", "is_main"]

# 仿真案例设置
StatusParams = [
    {'MissileInitStatus': {"t": 0,
                           "longitude": np.deg2rad(0),
                           "latitude": np.deg2rad(50),
                           "height": 80000,
                           "velocity": 7000,
                           "path_angle": np.deg2rad(0),
                           "heading_angle": np.deg2rad(120)},
     'TargetInitStatus': {"longitude": np.deg2rad(75),
                          "latitude": np.deg2rad(-25)},
     'MissileEndStatus': {
         "velocity": 2000,
         "height": 2.5e4,
         "s": 5e4,
         "heading_angle": None,
         "t": 2150}
     }
]

# 制导相关参数

# 是否考虑最大倾侧角
CONSIDER_SIGMA_MAX = True
# K_GAMMA = 3  # 下降段攻角反馈系数
K_GAMMA_SGP = 3  # 滑翔段TDCT反馈系数
ERR_TOL = 3e-4  # 进入平稳滑翔段时弹道倾角与平稳滑翔弹道倾角的容忍误差
K_SIGMA = 10  # 最大倾侧角中的系数
K_ALPHA = 5 * np.pi / 1E7 / 1.8
# 要从制导参数直接导入到控制参数的值名称↓
CONTROL_PARAM_LIST = ["m", "attack_angle", "bank_angle"]
# 除控制参数和状态外要从制导模块(missile.guide:dict)保存的数据
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
    'q',
    'L12D',
    'L12D_E',
    'L12D_alpha',
    'a_tol',
    'aL1',
    'aL2',
    'CL_beg',
    'aL2_orig',
    "ref_sgo",
    "ref_t",
    "ref_L12D",
    "L12D_beg"
    # "sigma_aap_aL2",
    # "sigma_aap_deltaL2m"
]
# 更新EBR2时最大迭代次数
MAX_EBR2_ITER = 5

# 保存数据
STORE_DATA = f'store/data_saved/multiSimulation_case.csv'
