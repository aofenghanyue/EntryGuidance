# -*- coding: utf-8 -*-
# EditTime  : 2021-09-29 16:17
# Author    : Of yue
# File      : MultiGuide.py
# Intro     :
from typing import List

from entity.missile import Missile
import multiset as glbs


class MultiMissileGuidance:
    def __init__(self):
        pass

    def init(self, missile: List[Missile], target: List[Missile] = [], meta={}):
        # 初始化
        for index, mis in enumerate(missile):
            # 各导弹是否终止制导
            mis.guide["end_guide"] = False
            mis.guide["index"] = index
            # 各导弹制导计时器
            mis.guide["time_pass"] = 0
            mis.guide["guide_process"] = True
            # 仿真步长初始值
            mis.guide["step_len"] = getattr(glbs, 'INI_STEP', 0.1)
        for t in target:
            # 静止目标
            t.guide["guide_process"] = False
            t.guide["end_guide"] = True

        self.init_custom(missile, target, meta)

    def init_custom(self, missile: List[Missile], target: List[Missile] = [], meta={}):
        pass

    def one_step_guide(self, missile: List[Missile], target: List[Missile] = [], meta={}):
        """
        具体制导流程
        :param meta: 附加参数
        :param missile: 导弹对象
        :param target: 目标对象
        :return: True代表制导成功，False代表制导结束
        """
        if self.end_guide(missile, target, meta):
            return False
        coop_data = self.coop_guide(missile, target, meta)
        if coop_data:
            meta["coop_data"] = coop_data
        for mis, tar in zip(missile, target):
            # 判断是否发射且未结束飞行
            if self.is_launched(mis, tar, meta) and not mis.guide["end_guide"]:
                # 判断是否制导
                if self.guide_process(mis, tar, meta):
                    # 单步制导
                    self.guide(mis, tar, meta)
        return True

    def coop_guide(self, missile: List[Missile], target: List[Missile] = [], meta={}):
        # 协同参数，默认无，如果有协同参数，则建议使用字典形式传出
        return False

    def guide(self, missile: Missile, target: Missile = None, meta={}):
        """
        在这个方法中得到导弹和目标的控制指令
        :param missile:
        :param target:
        :param meta:
        :return:
        """
        pass

    def is_launched(self, missile: Missile, target: Missile = None, meta={}):
        """
        判断各导弹是否发射，如果发射了，则将missile.launched赋值为True
        :param missile:
        :param target:
        :param meta:
        :return:
        """
        return missile.launched

    def guide_process(self, missile: Missile, target: Missile, meta={}):
        """
        判断这个制导周期是否计算制导指令，
        使用情况：每隔1秒更新制导指令，但积分步长为0.1秒，可以由此判断
        :param meta:
        :param missile:
        :param target:
        :return: True 代表此周期计算制导指令，False代表延续上一周期的指令
        """
        # 如果导弹没发射，则不进入制导循环
        if not missile.launched:
            return False
        # 下面是每隔step_len制导一次的例子
        if missile.guide["time_pass"] == 0:
            missile.guide["guide_process"] = True
        else:
            missile.guide["guide_process"] = False
        missile.guide["time_pass"] += meta["min_h"]
        if abs(missile.guide["time_pass"] - missile.guide["step_len"]) < 1e-9:
            missile.guide["time_pass"] = 0
        return missile.guide["guide_process"]

    def end_guide(self, missile: List[Missile] = [], target: List[Missile] = [], meta={}):
        """
        是否结束制导
        例如已经打到目标或者达到最大制导时长，则终止制导
        :param missile:
        :param target:
        :param meta:
        :return:
        """
        for m, t in zip(missile, target):
            self.mis_end_guide(m, t, meta)
        end_flags = [mis.guide["end_guide"] for mis in missile]
        if False in end_flags:
            return False
        else:
            return True

    def mis_end_guide(self, mis: Missile, tar: Missile, meta={}):
        pass
