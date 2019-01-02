import numpy as np 
import copy
import pinocchio as se3
from task_motion import *
from ..Trajectories import TrajectorySample
from ..Math import ConstraintEquality
from ..Utils import utils

class TaskSE3Equality(TaskMotion):
    def __init__(self, name, robot, frameName):
        TaskMotion.__init__(self, name, robot)
        self.m_robot = robot
        self.m_frameName = frameName
        self.m_constraint = ConstraintEquality(name, 6, robot.nv) # name / rows / cols
        self.m_ref = TrajectorySample(12, 6)

        assert self.m_robot.model.exitFrame(frameName)
        
        self.m_frame_id = self.m_robot.model.getFrameId(frameName)
        self.m_v_ref = se3.Motion() # m_v_ref > se3::Motion
        self.m_v_ref.setZero()
        self.m_a_ref = se3.Motion()
        self.m_a_ref.setZero()
        self.m_M_ref = se3.SE3()
        self.m_M_ref.setIdentity()
        self.m_wMl = copy.deepcopy(self.m_M_ref)
        self.m_wMl.setIdentity()
        self.m_p_error_vec = np.matrix(np.zeros(6)).transpose()
        self.m_v_error_vec = np.matrix(np.zeros(6)).transpose()
        self.m_p = np.matrix(np.zeros(12)).transpose()
        self.m_v = np.matrix(np.zeros(6)).transpose()
        self.m_p_ref = np.matrix(np.zeros(12)).transpose()
        self.m_v_ref_vec = np.matrix(np.zeros(6)).transpose()
        self.m_Kp = np.matrix(np.zeros(6)).transpose()
        self.m_Kd = np.matrix(np.zeros(6)).transpose()
        self.m_a_des = np.matrix(np.zeros(6)).transpose()
        self.m_J = np.matrix(np.zeros((6, robot.nv))).transpose()
        self.m_drift = []
        self.m_position_only = False

    def dim(self):
        if not self.m_position_only:
            return 6
        else:
            return 3

    def compute(self, t, q, v, data):
        oMi = self.m_robot.framePosition(data, self.m_frame_id)
        v_frame = self.m_robot.frameVelocity(data, self.m_frame_id)
        self.m_drift = self.m_robot.frameClassicAcceleration(self.m_frame_id)

        self.m_wMl.rotation = copy.deepcopy(oMi.rotation)
        self.m_p_error = se3.log6FromSE3(self.m_M_ref.inverse()*oMi) 
        self.m_v_error = v_frame - self.m_wMl.actInv(self.m_v_ref) 

        self.m_p_error_vec = self.m_p_error.vector # C++ > .toVector()
        self.m_v_error_vec = self.m_v_error.vector

        self.m_p_ref = utils.SE3toVector(self.m_M_ref)
        self.m_v_ref_vec = self.m_v_ref.vector
        self.m_p = utils.SE3toVector(oMi)
        self.m_v = v_frame.vector

        # Get desired acc 
        for i in range(0,6):
            self.m_a_des[i] = - self.m_Kp[i]*self.m_p_error_vec[i] - self.m_Kd[i]*self.m_v_error_vec[i] + self.m_wMl.actInv(self.m_a_ref).vector[i]

        self.m_J = self.m_robot.frameJacobianLocal(q, self.m_frame_id, se3.ReferenceFrame.LOCAL)
        if not self.m_position_only:
            self.m_constraint.setMatrix(self.m_J)
            self.m_constraint.setVector(self.m_a_des - self.m_drift.vector)
        else : 
            self.m_constraint.setMatrix(self.m_J[0:3,])
            self.m_constraint.setVector(self.m_a_des[0:3]- self.m_drift.vector[0:3])

        return self.m_constraint


    def getConstraint(self):
        return self.m_constraint    

    def setReference(self, ref):
        self.m_ref = ref
        self.m_M_ref = utils.vectorToSE3(ref.pos)
        self.m_v_ref = se3.Motion(ref.vel)
        self.m_a_ref = se3.Motion(ref.acc)

    def getReference(self):
        return self.m_ref

    def position_error(self):
        return self.m_p_error_vec

    def velocity_error(self):
        return self.m_v_error_vec

    def position(self):
        return self.m_p

    def position_ref(self):
        return self.m_p_ref   

    def velocity(self):
        return self.m_v

    def velocity_ref(self):
        return self.m_v_ref

    def Kp(self):
        return self.m_Kp

    def Kd(self):
        return self.m_Kd
        
    def setKp(self, Kp):
        assert(len(Kp)==6)
        self.m_Kp = Kp

    def setKd(self, Kd):
        assert(len(Kd)==6)
        self.m_Kd = Kd          

    def getDesiredAcceleration(self):
        return self.m_a_des

    def getAcceleration(self, dv):
        return self.m_constraint.matrix() * dv + self.m_drift.vector



