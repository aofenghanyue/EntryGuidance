from entity.missile import Missile


class Guidance:
    def __init__(self):
        super(Guidance, self).__init__()
        self.end_flag = False

    def init(self, missile: Missile, target: Missile = None, meta={}):
        pass

    def one_step_guide(self, missile: Missile, target: Missile = None, meta={}):
        """
        具体制导流程
        :param meta: 附加参数
        :param missile: 导弹对象
        :param target: 目标对象
        :return: True代表制导成功，False代表制导结束
        """
        if not self.end_guide(missile, target, meta) and not self.end_flag:
            if self.guide_process(missile, target, meta):
                self.guide(missile, target, meta)
            else:
                pass
            return True
        else:
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

    def guide_process(self, missile: Missile, target: Missile, meta={}):
        """
        判断这个制导周期是否计算制导指令，
        使用情况：每隔1秒更新制导指令，但积分步长为0.1秒，可以由此判断
        :param meta:
        :param missile:
        :param target:
        :return: True 代表此周期计算制导指令，False代表延续上一周期的指令
        """
        return True

    def end_guide(self, missile: Missile=Missile(), target: Missile=Missile(), meta={}, flag=False):
        """
        是否结束制导
        例如已经打到目标或者达到最大制导时长，则终止制导
        :param missile:
        :param target:
        :param meta:
        :return:
        """
        if self.end_flag:
            return True
        ##
        # your code at guide end
        ##
        if flag:
            self.end_flag = flag
        return self.end_flag
