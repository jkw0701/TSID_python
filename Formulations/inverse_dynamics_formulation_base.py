import pinocchio as se3
import numpy as np 

class InvdynBase(object):
    def __init__(self, name, robot, verbose=False):
        self.m_name = name
        self.m_robot = robot
        self.m_verbose = verbose

    def data(self):
        return 0

    def nVar(self):
        return 0
    
    def nEq(self):
        return 0
    
    def nIn(self):
        return 0

    def addMotionTask(self, task, weight, priority, transition_time = 0.0):
        return 0

    def addForceTask(self, task, weight, priority, transition_time = 0.0):
        return 0
        
    def addTorqueTask(self, task, weight, priority, transition_time = 0.0):
        return 0     

    def updateTaskWeight(self, task_name, weight):
        return 0

    def addRigidContact(self, contact):
        return 0            

    def removeTask(self, task_name, transition_time = 0.0):
        return 0             

    def removeRigidContact(self, contact_name, transition_time = 0.0):
        return 0             

    def computeProblemData(self, time, q, v):
        return 0                     

    def getActuatorForces(self, sol):
        return 0             

    def getAccelerations(self, sol):
        return 0     

    def getContactForces(self, sol):
        return 0     
    
    def getContactForce(self, name, sol, f):
        return 0     