import numpy as np 
import copy
from constraint_base import ConstraintBase

class ConstraintEquality(ConstraintBase):
    def __init__(self, *args):
        if len(args) == 1: # only name
            ConstraintBase.__init__(self, args[0])
        elif len(args) == 2: # null
            assert len(args) == 2
        elif len(args) == 3: # name & rows & cols
            if isinstance(args[1], np.ndarray) and isinstance(args[2], np.ndarray):
                # isinstance(A,B) : A 가 B 형식 인지 true or false
                ConstraintBase.__init__(self, args[0], args[1])
                self.m_b = args[2]
                assert len(self.m_A) == len(self.m_b)
            elif isinstance(args[1], int) and isinstance(args[2], int):
                ConstraintBase.__init__(self, args[0], args[1], args[2])
                self.m_b = np.matrix(np.zeros(args[1])).transpose()
            else:
                assert False

        assert len(args) < 4

    def rows(self):
        assert len(self.m_A) == len(self.m_b)
        return len(self.m_A)

    def cols(self):
        return self.m_A.size / self.rows()
    
    def resize(self, r, c): # rows & columns
        assert r == c 
        self.m_A = np.matrix(np.identity((r,c)))
        self.m_b = np.matrix(np.zeros(r)).transpose()

    def isEquality(self):
        return True

    def isInequality(self):
        return False

    def isBound(self):
        return False

    def vector(self):
        return self.m_b

    def lowerBound(self):
        assert False
        return self.m_b

    def upperBound(self):
        assert False
        return self.m_b

    def setVector(self, b):
        self.m_b = copy.deepcopy(b)
        return True


    def setLowerBound(self, lb):
        assert False
        return False

    def setUpperBound(self, ub):
        assert False
        return False



        