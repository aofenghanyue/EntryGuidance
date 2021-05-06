from entity.missile import Missile
from guidance.guide import Guidance
from simulation import TrajectorySimulation
from store.dataSave import DataSave
from utils.integral import Integral


class CustomSimulation(TrajectorySimulation):
    def __init__(self):
        super(CustomSimulation, self).__init__()
    
    def init(self, mis=Missile(), tar=Missile(), guide=Guidance(), integ=Integral(), db=DataSave()):
        self.mis = mis
        self.tar = tar
        self.guide = guide
        self.integral = integ
        self.db = db
        
    def step_len(self) -> float:
        return 1

    def save_data(self) -> None:
        self.db_save_dict = {"global_t": self.t}
        self.db_save_dict.update(self.mis.status.status_dict())
        self.db_save_dict.update(self.mis.control)

        self.db.update(self.db_save_dict)

    def is_continue(self):
        return True