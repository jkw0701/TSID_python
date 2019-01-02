import numpy as np 
from trajectory_base import *

class TrajectoryEuclidianConstant(TrajectoryBase):
    def __init__(self, name, RefVec=0):
        self.m_name = name
        self.m_ref = RefVec
        self.m_sample = TrajectorySample(len(RefVec)) # class

        if not len(RefVec) == 0 :
            self.setReference(self.m_ref)

    def size(self):
        return self.m_sample.pos.size

    def setReference(self, ref): # ref : position
        self.m_sample.setPos(ref)
        self.m_sample.setVel(np.matrix(np.zeros(len(ref))).transpose())        
        self.m_sample.setAcc(np.matrix(np.zeros(len(ref))).transpose())

    def computeNext(self):
        return self.m_sample

    def getLastSample(self):
        return self.m_sample

    def has_trajectory_ended(self):
        return True            
