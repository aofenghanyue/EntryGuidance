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

"""初始化"""
# 1. 创建导弹对象
# 1.1 状态及初始化
mis_stat = ME6DStatus()
mis_stat.change_stat(glbs.MissileInitStatus)
# 1.2 创建动力学微分方程
mis_eq = ME6D()
# 1.3 创建气动参数模块
mis_aerodynamic = AerodynamicCAVH()
# 1.4 导弹固有参数
mis_param = CAVHParams()
# 1.5 创建导弹
mis = Missile(motion_equation=mis_eq,
              aerodynamic=mis_aerodynamic,
              params=mis_param,
              status=mis_stat)
# 2. 创建目标对象
tar_stat = MissileStatus()
tar_stat.change_stat(glbs.TargetInitStatus)
tar = Missile(status=tar_stat)
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

result = simulation.db.data
print(result)
