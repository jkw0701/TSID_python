import numpy as np 
import copy
from task_motion import *
from ..Trajectories import TrajectorySample
from ..Math import ConstraintEquality

class TaskJointPosture(TaskMotion):
    def __init__(self, name, robot):
        TaskMotion.__init__(self, name, robot)
        self.m_robot = robot
        if robot.isFloatingBase():
            self.m_ref = TrajectorySample(self.m_robot.nv - 6) # why minus 6 ?? Ans : floating base
            self.m_constraint = ConstraintEquality(name, robot.nv -6, robot.nv)
            self.m_Kp = np.matrix(np.zeros(robot.nv-6)).transpose()
            self.m_Kd = np.matrix(np.zeros(robot.nv-6)).transpose()
            self.m_a_des = np.matrix(np.zeros(robot.nv-6)).transpose()
            self.m_mask = []
            m = np.matrix(np.ones(robot.nv-6), np.int).transpose()
            self.setMask(m)
        else:
            self.m_ref = TrajectorySample(self.m_robot.nv) # why minus 6 ?? Ans : floating base
            self.m_constraint = ConstraintEquality(name, robot.nv, robot.nv)
            self.m_Kp = np.matrix(np.zeros(robot.nv)).transpose()
            self.m_Kd = np.matrix(np.zeros(robot.nv)).transpose()
            self.m_a_des = np.matrix(np.zeros(robot.nv)).transpose()
            self.m_mask = []
            m = np.matrix(np.ones(robot.nv), np.int).transpose()
            self.setMask(m)

    def dim(self):
        return int(self.m_mask.sum())

    def compute(self, t, q, v):
        if self.m_robot.isFloatingBase():
            self.m_p = q[6:]
            self.m_v = v[6:]
            self.m_p_error = self.m_p - self.m_ref.pos 
            self.m_v_error = self.m_v - self.m_ref.vel 

            for i in range(0, self.m_robot.nv - 6):
                self.m_a_des[i] = - self.m_Kp[i]*self.m_p_error[i] - self.m_Kd[i]*self.m_v_error[i] + self.m_ref.acc[i]

            for i in range(0, len(self.m_activeAxes)):
                self.m_constraint.vector()[i] = self.m_a_des[self.m_activeAxes[i]]

            return self.m_constraint

        else:
            self.m_p = q
            self.m_v = v
            self.m_p_error = self.m_p - self.m_ref.pos 
            self.m_v_error = self.m_v - self.m_ref.vel 

            for i in range(0, self.m_robot.nv):
                self.m_a_des[i] = - self.m_Kp[i]*self.m_p_error[i] - self.m_Kd[i]*self.m_v_error[i] + self.m_ref.acc[i]

            for i in range(0, len(self.m_activeAxes)):
                self.m_constraint.vector()[i] = self.m_a_des[self.m_activeAxes[i]]

            return self.m_constraint

    def getConstraint(self):
        return self.m_constraint

    def setReference(self, ref):
        if self.m_robot.isFloatingBase():
            assert( len(ref.pos)== self.m_robot.nv - 6)
            assert( len(ref.vel)== self.m_robot.nv - 6)
            assert( len(ref.acc)== self.m_robot.nv - 6)
            self.m_ref = ref
        else:
            assert( len(ref.pos)== self.m_robot.nv)
            assert( len(ref.vel)== self.m_robot.nv)
            assert( len(ref.acc)== self.m_robot.nv)
            self.m_ref = ref

    def getReference(self):
        return self.m_ref

    def getDesiredAccleration(self):
        return self.m_a_des

    def getAcceleration(self,dv):
        return self.m_constraint.matrix()*dv
        
    def mask(self):
        return self.m_mask

    def setMask(self, m):
        if self.m_robot.isFloatingBase():
            self.m_mask = m
            dim = self.dim()
            S = np.matrix(np.zeros((dim, self.m_robot.nv)))
            self.m_activeAxes = np.matrix(np.zeros(dim), np.integer).transpose()
            j = 0
            for i in range(0, len(m)):
                if m[i,0] != 0 :
                    assert m[i] == 1.0
                    S[j, i] = 1.0
                    self.m_activeAxes[i] = i
                    j = j+1

            self.m_constraint.resize(dim, self.m_robot.nv)
            self.m_constraint.setMatrix(S)


    def position_error(self):
        return self.m_p_error

    def velocity_error(self):
        return self.m_v_error

    def position(self):
        return self.m_p

    def position_ref(self):
        return self.m_ref.pos

    def velocity(self):
        return self.m_v

    def velocity_ref(self):
        return self.m_ref.vel

    def Kp(self):
        return self.m_Kp

    def Kd(self):
        return self.m_Kd

    def setKp(self, Kp):
        if self.m_robot.isFloatingBase():
            assert(len(Kp)==self.m_robot.nv - 6)
            self.m_Kp = Kp
        else :
            assert(len(Kp)==self.m_robot.nv)
            self.m_Kp = Kp

    def setKd(self, Kd):
        if self.m_robot.isFloatingBase():
            assert(len(Kd)==self.m_robot.nv -6)
            self.m_Kd = Kd
        else:
            assert(len(Kd)==self.m_robot.nv)
            self.m_Kd = Kd            




