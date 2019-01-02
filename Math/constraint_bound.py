import numpy as np 
import copy
from constraint_base import ConstraintBase

class ConstraintBound(ConstraintBase):
    def __init__(self, *args):
        if len(args) == 1: # only name
            ConstraintBase.__init__(self, args[0])
        elif len(args) == 2: # name & size
            ConstraintBase.__init__(self, args[0], np.matrix(np.identity(args[1])))
            self.m_lb = np.matrix(np.zeros(args[1])).transpose()
            self.m_ub = np.matrix(np.zeros(args[1])).transpose()
        elif len(args) == 3: # name & bounds
            ConstraintBase.__init__(self, args[0], np.matrix(np.identity(args[1].size)))    
            self.m_lb = args[1]
            self.m_ub = args[2]
            assert len(self.m_lb) == len(self.m_ub)

        assert len(args) < 4


    def rows(self):
        assert len(self.m_lb) == len(self.m_ub)
        return len(self.m_lb)

    def cols(self):
        assert len(self.m_lb)  == len(self.m_ub)
        return len(self.m_lb)
    
    def resize(self, r, c): # rows & columns
        assert r == c 
        self.m_A = np.matrix(np.identity((r,c)))
        self.m_lb = np.matrix(np.zeros(r)).transpose()
        self.m_ub = np.matrix(np.zeros(r)).transpose()

    def isEquality(self):
        return False

    def isInequality(self):
        return False

    def isBounde(self):
        return True

    def vector(self):
        assert False
        return self.m_lb

    def lowerBound(self):
        return self.m_lb

    def upperBound(self):
        return self.m_ub

    def setVector(self):
        assert False
        return False

    def setLowerBound(self, lb):
        self.m_lb = copy.deepcopy(lb)
        return True

    def setUpperBound(self, ub):
        self.m_ub = copy.deepcopy(ub)
        return True



