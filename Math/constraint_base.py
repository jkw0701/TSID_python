import numpy as np 
import copy
# copy : shallow & deep 
# self : instance of class or function

class ConstraintBase(object):
    def __init__(self, *args):
        if len(args) == 1: # len > # of components of args
            self.m_name = args[0]
        elif len(args) == 2:
            self.m_name = args[0]
            self.m_A = args[1]
        elif len(args) == 3:
            self.m_name = args[0]
            self.m_A = np.matrix(np.zeros((args[1], args[2])))

        assert len(args) < 4

    def name(self):
        return self.m_name

    def matrix(self):
        return self.m_A

    def setMatrix(self, A):
        self.m_A = A
        return True

    def setMatrix_middlecols(self, a, b, A):
        self.m_A[a:a+b,:] = A
        return True        

        

