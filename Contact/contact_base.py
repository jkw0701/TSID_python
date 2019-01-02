import numpy as np 
import copy

class ContactBase(object):
    def __init__(self, name, robot):
        self.m_name = name
        self.m_robot = robot
        self.m_n_motion = 0
        self.m_n_force = 0

    def name(self, name):
        return self.m_name

        

