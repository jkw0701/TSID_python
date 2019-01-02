import numpy as np 
import copy

class TaskBase(object):
    def __init__(self, name, robot):
        self.m_name = name
        self.m_robot = robot

    def name(self, arg = 0):
        if arg == 0:
            return self.m_name
        else : 
             self.m_name == arg

    def dim(self):
        return 0

    def compute(self, t, q, v):
        return 0

    def getConstraint(self):
        return 0    
