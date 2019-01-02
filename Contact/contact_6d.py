from ..Utils import *
import pinocchio as se3
import numpy as np
import copy
from contact_base import ContactBase
from ..Tasks import TaskSE3Equality
from ..Math import *
from ..Trajectories import TrajectorySample

class Contact6d(ContactBase):
    def __init__(self, name, robot, frameName, contactPoints, contactNormal, frictioncoefficient, minNormalForce, maxNormalForce, regularizationTaskWeight):
        ContactBase.__init__(self, name, robot)
        self.m_motionTask = TaskSE3Equality(name, robot, frameName) 
        self.m_forceInequality = ConstraintInequality(name, 16, 12)
        self.m_forceRegTask = ConstraintEquality(name, 12, 12)
        self.m_contactPoints = contactPoints
        self.m_contactNormal = contactNormal
        self.m_mu = frictioncoefficient
        self.m_fMin = minNormalForce
        self.m_fMax = maxNormalForce
        self.m_regularizationTaskWeight = regularizationTaskWeight
        self.m_weightForceRegTask = np.array([1, 1, 1e-3, 2, 2, 2])
        self.m_forceGenMat = np.matrix(np.zeros((6, 12)))
        self.m_fRef = np.matrix(np.zeros(6)).transpose()
        self.m_ref = TrajectorySample(12, 6)

        self.updateForceGeneratorMatrix()
        self.updateForceInequalityConstraints()
        self.updateForceRegularizationTask()

    def updateForceInequalityConstraints(self):
        t1 = np.matrix(np.zeros(3))
        t2 = np.matrix(np.zeros(3))
        
        self.m_contactNormal = np.array([0.0, 0.0, 1.0])
        self.m_mu = 0.01

        n_in = 4*4 + 1
        n_var = 3*4

        B = np.matrix(np.zeros((n_in, n_var))) # 17 X 12

        lb = -1e10*np.matrix(np.ones(n_in)).T
        ub = np.matrix(np.zeros(n_in)).T

        t1 = np.cross(self.m_contactNormal, [1.0, 0.0, 0.0])
    
        if np.linalg.norm(t1, 2) < 1e-5:
            t1 = np.cross(self.m_contactNormal, [0, 1, 0])

        t2 = np.cross(self.m_contactNormal, t1)

        t1 = t1 / np.linalg.norm(t1, 2)
        t2 = t2 / np.linalg.norm(t2, 2)

        B[0,0:3] = (-t1 - self.m_mu*self.m_contactNormal)
        B[1,0:3] = (t1 - self.m_mu*self.m_contactNormal)
        B[2,0:3] = (-t2 - self.m_mu*self.m_contactNormal)
        B[3,0:3] = (t2 - self.m_mu*self.m_contactNormal)

        for i in range(1, 4):
            B[4*i:4*i+4,3*i:3*i+3] = B[0:4,0:3] 
            B[n_in-1,3*(i-1):3*i] = self.m_contactNormal

        B[n_in-1,9:12] = self.m_contactNormal

        lb[n_in-1] = self.m_fMax
        ub[n_in-1] = self.m_fMin

        self.m_forceInequality.setMatrix(B)
        self.m_forceInequality.setLowerBound(lb)
        self.m_forceInequality.setUpperBound(ub)
        
    def getNormalForce(self, f):
        assert len(f) == 12
        n = 0.0
        for i in range(0, 4):
            n += np.dot(self.m_contactNormal,f[i*3:(i+1)*3])    

        return n

    def setRegularizationTaskWeightVector(self, w):
        self.m_weightForceRegTask = w
        self.updateForceRegularizationTask()

    def updateForceRegularizationTask(self):
        A = np.matrix(np.zeros((6, 6)))

        for i in range(0,6):
            A[i,i] = self.m_weightForceRegTask[i]

        self.m_forceRegTask.setMatrix(A*self.m_forceGenMat)    
        self.m_forceRegTask.setVector(self.m_fRef)    

    def updateForceGeneratorMatrix(self): # Force / Torque Conversion Matrix
        assert self.m_contactPoints.rows() == 3
        assert self.m_contactPoints.cols() == 4

        for i in range(0, 4):
            self.m_forceGenMat[0:3,3*i:3*(i+1)] = np.eye(3)
            self.m_forceGenMat[3:6,3*i:3*(i+1)] = utils.skew(self.m_contactPoints[0:3,i])

    def n_motion(self):
        return 6

    def n_force(self):
        return 12

    def Kp(self):
        return self.m_motionTask.Kp()

    def Kd(self):
        return self.m_motionTask.Kd()

    def setKp(self, Kp):
        assert(len(Kp)==6)
        self.m_Kp = Kp

    def setKv(self, Kv):
        assert(len(Kv)==6)
        self.m_Kv = Kv          

    def setContactPoints(self, contactPoints):
        assert contactPoints.rows() == 3
        assert contactPoints.cols() == 4
        if contactPoints.rows()!= 3 or contactPoints.cols()!=4:
            return False
        self.m_contactPoints = contactPoints
        self.updateForceGeneratorMatrix()
        return True

    def setContactNormal(self, contactNormal):
        assert len(contactNormal) == 3  # fix me!
        if(len(contactNormal)!=3):
            return False
        self.m_contactNormal = contactNormal
        self.updateForceInequalityConstraints()    
        return True
    
    def setFrictionCoefficient(self, frictionCoefficient):
        assert frictionCoefficient>0.0
        if frictionCoefficient < 0.0 :
            return False
        self.m_mu = frictionCoefficient
        self.updateForceInequalityConstraints()
        return True

    def setMinNormalForce(self, minNormalForce):
        assert minNormalForce > 0.0 and minNormalForce < self.m_fMax
        if minNormalForce <= 0 and minNormalForce > self.m_fMin:
            return False

        self.m_fMin = minNormalForce
        lb = copy.deepcopy(self.m_forceInequality.lowerBound()) # lb is reference of lowerbound
        lb[len(lb)-1] = self.m_fMin
        return True   

    def setMaxNormalForce(self, maxNormalForce):
        assert maxNormalForce >= self.m_fMin
        if maxNormalForce < self.m_fMin:
            return False

        self.m_fMax = maxNormalForce
        ub = copy.deepcopy(self.m_forceInequality.upperBound())
        ub[len(ub)-1] = self.m_fMax
        return True    

    def setRegularizationTaskWeight(self, w):
        assert w >= 0.0
        if w < 0.0:
            return False
        self.m_regularizationTaskWeight = w
        return True    
 
    def setForceReference(self, f_ref):
        self.m_fRef = f_ref
        self.updateForceRegularizationTask()

    def setReference(self, ref):
        self.m_ref.pos = utils.SE3toVector(ref)
        self.m_motionTask.setReference(self.m_ref)

    def computeMotionTask(self, t, q, v, data):
        return self.m_motionTask.compute(t, q, v, data)

    def computeForceTask(self):
        return self.m_forceInequality    

    def getForceGeneratorMatrix(self):
        return self.m_forceGenMat
    
    def computeForceRegularizationTask(self):
        return self.m_forceRegTask

    def getMinNormalForce(self):
        return self.m_fMin

    def getMaxNormalForce(self):
        return self.m_fMax

    def getMotionTask(self):
        return self.m_motionTask

    def getMotionConstraint(self):
        return self.m_motionTask.getConstraint()

    def getForceConstraint(self):
        return self.m_forceInequality

    def getForceRegularizationTask(self):
        return self.m_forceRegTask    

    def getForceRegularizationWeight(self):
        return self.m_regularizationTaskWeight
