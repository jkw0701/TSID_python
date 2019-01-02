import numpy as np 
import copy 

class TrajectorySample(object):
    def __init__(self, size_pos, size_vel=0):
        self.pos = []
        self.vel = []
        self.acc = []

        if size_vel == 0:
            self.resize(size_pos,size_pos)
        else:
            self.resize(size_pos, size_vel)

    def resize(self, size_pos, size_vel):
        self.pos = np.matrix(np.zeros(size_pos)).transpose()
        self.vel = np.matrix(np.zeros(size_vel)).transpose()
        self.acc = np.matrix(np.zeros(size_vel)).transpose()

    def setPos(self, pos):
        self.pos = copy.deepcopy(pos)

    def setVel(self, vel):
        self.vel = copy.deepcopy(vel)

    def setAcc(self, acc):
        self.acc = copy.deepcopy(acc)

class TrajectoryBase(object):
    def __init__(self, name):
        self.m_name = name
        self.m_size = 0
        self.m_sample = []        

    def size(self):
        return self.size

    def computeNext(self):
        return self.m_sample

    def getLastSample(self):
        return self.m_sample

    def has_trajectory_ended(self):
        return True    


