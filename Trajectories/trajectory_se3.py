import numpy as np
from pinocchio import se3 
from trajectory_base import *
from ..Utils import *

class TrajectorySE3Constant(TrajectoryBase):
    def __init__(self, name, M):
        self.m_name = name
        self.m_sample = TrajectorySample(12, 6) # class

    def size(self):
        return 6

 #   def setReference(self, ref): # ref : position
 #       self.m_sample.setPos(ref)
 #       ref = utils.SE3toVector(ref)        

    def computeNext(self):
        return self.m_sample

    def getLastSample(self):
        return self.m_sample

    def has_trajectory_ended(self):
        return True            
