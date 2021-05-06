from entity.missile import Missile
from guidance.guide import Guidance


class CustomGuidance(Guidance):
    def __init__(self):
        super(CustomGuidance, self).__init__()

    def guide(self, missile: Missile, target: Missile = None, meta={}):
        pass