import pandas

import settings as glbs
from customGuidance import CustomGuidance
from customSimulation import CustomSimulation
from database.vehicleParams import CAVHParams
from dynamics.aerodynamic import AerodynamicCAVH
from dynamics.motionEquation import ME6D
from entity.missile import Missile
from store.dataSave import DataSave
from store.status import ME6DStatus, MissileStatus
from utils.integral import RungeKutta4
from graphics.origin import debug_origin

"""初始化"""
# 1. 创建导弹对象
# 1.1 创建导弹状态
# 1.2 创建动力学微分方程
# 1.3 创建气动参数模块
# 1.4 导弹固有参数
# 1.5 创建导弹
mis = Missile(motion_equation=ME6D(),
              aerodynamic=AerodynamicCAVH(),
              params=CAVHParams(),
              status=ME6DStatus())
# 2. 创建目标对象
tar = Missile(status=MissileStatus())
# 3. 创建制导模块
guide = CustomGuidance()
# 4. 创建积分模块
integral = RungeKutta4()
# 5. 创建数据存储模块
database = DataSave()

"""创建仿真模型"""
simulation = CustomSimulation()
simulation.init(mis=mis, tar=tar, guide=guide, integ=integral, db=database)

"""开始仿真"""
simulation.simulation()

"""保存结果"""
result: pandas.DataFrame = simulation.db.data
result.to_csv(path_or_buf=glbs.STORE_DATA)
print(f'结果数据已保存至{glbs.STORE_DATA}')

"""处理数据"""
debug_origin(result)
# 3.7版本后breakpoint要加括号
# breakpoint()
