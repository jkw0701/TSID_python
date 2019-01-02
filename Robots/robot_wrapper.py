from pinocchio.robot_wrapper import RobotWrapper as PinocchioRobotWrapper
import pinocchio as se3
import numpy as np 

class RobotWrapper(PinocchioRobotWrapper):
    def __init__(self, filename, vector, joint_model, verbose):
        self.m_model = se3.buildModelFromUrdf(filename,joint_model)


    def com_all(self): # input com data & return
        com_tmp = []
        com_tmp.append(self.data.com[0])
        com_tmp.append(self.data.vcom[0])
        com_tmp.append(self.data.acom[0])
        return com_tmp

    def ifFloatingBase(self):
        if self.nv == self.nq: # fixed base
            return False
        else :
            return True

    def frameClassicAcceleration(self, index):
        f = self.m_model.frames[index]
        a = f.placement.actInv(self.data.a[f.parent])
        v = f.placement.actInv(self.data.v[f.parent])
        a.linear += np.cross(v.angular.T, v.linear.T).T
        return a

    def frameJacobianLocal(self, q, frame_id, LOCAL):
        m_J = se3.getFrameJacobian(self.m_model, q, frame_id, LOCAL)
        return m_J 

    def framePosition(self, data, index):
        f = self.m_model.frames[index]
        return self.data.oMi[f.parent].act(f.placement)
 
    def computeAllTerms(self, data, q, v):
        se3.computeAllTerms(self.m_model, data, q, v)
         
    def mass(self, data):
        return data.M

    def nonLinearEffects(self, data):
        return data.nle    




