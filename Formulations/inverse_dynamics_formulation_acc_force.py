import pinocchio as se3
import numpy as np
from inverse_dynamics_formulation_base import * # import all
from ..Math import *
from ..Solvers import ConstraintLevel, HQPData
from ..Contact import ContactBase

class TaskLevel(object):
    def __init__(self, task, priority): # task : Taskbase
        self.m_priority = priority
        self.m_task = task
        self.m_constraint = []

    def setConstraint(self, const):
        self.m_constraint = const

    def getConstraint(self):
        return self.m_constraint

    def getTask(self):
        return self.m_task

class ContactLevel(object):
    def __init__(self, ContactBase): # ContactBase
        self.m_contact = ContactBase
        self.m_index = 0
        self.m_motionConstraint = ConstraintBase() # instances of class
        self.m_forceConstraint = ConstraintInequality()
        self.m_forceRegTask = ConstraintEquality()

    def getContact(self):
        return self.m_contact

class ContactTransitionInfo(object):
    def __init__(self):
        self.m_time_start = 0
        self.m_time_end =0
        self.m_fMax_start = 0
        self.m_fMax_end = 0
        self.m_contactLevel = [] # list for ContactLevel

class Invdyn(InvdynBase):
    def __init__(self, name, robot, verbose=False):
        InvdynBase.__init__(self, name, robot, verbose)
        self.m_data = robot.model # why not m_robot ?? cuz IndynBase also has it
        if self.m_robot.isFloatingBase():
            self.m_baseDynamics = ConstraintEquality("base-dynamics", 6, self.m_robot.nv)

        self.m_solutionDecoded = False
        self.m_t = 0.0
        self.m_v = self.m_robot.nv 
        self.m_k = 0
        if self.m_robot.isFloatingBase():
            self.m_eq = 6
        else :
            self.m_eq = 0
        self.m_in = 0
        self.m_hqpData = HQPData() # from import
        self.m_hqpData.resize(3)
        self.m_Jc = np.matrix(np.zeros((self.m_k, self.m_v)))

        if self.m_robot.isFloatingBase():
            self.m_hqpData.setData(0, 1.0, self.m_baseDynamics) # append class?

        # lists of classes
        self.m_taskMotions =[] # std::vector<TaskLevel*> 
        self.m_taskContactForces = [] # std::vector<TaskLevel*> 
        self.m_taskActuations = []  # std::vector<TaskLevel*> 
        self.m_contactLevel = []
        self.m_contactTransitions = [] # std::vector<ContactTransitionInfo*>
        self.m_contacts = [] # std::vector<ContactLevel*>
        self.m_f =  np.matrix(np.zeros(6)).transpose()

    def data(self):
        return self.m_data

    def nVar(self):
        return self.m_v + self.m_k

    def nEq(self):
        return self.m_eq    

    def nIn(self):
        return self.m_in

    def resizeHqpData(self):
        self.m_Jc = np.matrix(np.zeros((self.m_k, self.m_v)))
        if self.m_robot.isFloatingBase():
            self.m_baseDynamics = ConstraintEquality("base-dynamics", 6, self.m_k + self.m_v) # m_A, m_b

        for i in range(0, len(self.m_hqpData.m_HQPData)):
            self.m_hqpData.m_HQPData[i][1].resize(self.m_hqpData.m_HQPData[i][1].rows(), self.m_k + self.m_v)
            # resize function in constraint-equality & inequality Clss

    def addTask(self, tl, weight, priority): # add hqpData & tasklevel
        if priority > len(self.m_hqpData.m_HQPData):
            for i in range(0, (priority-len(self.m_hqpData.m_HQPData))) :
                self.m_hqpData.m_HQPData.append([])
            
        c = tl.getTask().getConstraint() # tl : task level 

        if c.isEquality():
            tl.setConstraint(ConstraintEquality(c.name, c.rows(), self.m_v + self.m_k))
            if priority == 0 and self.m_robot.isFloatingBase() :
                self.m_eq += c.rows()
        elif c.isInequality():
            tl.setConstraint(ConstraintInequality(c.name, c.rows(), self.m_v + self.m_k))        
            if priority == 0 and self.m_robot.isFloatingBase() :
                self.m_in += c.rows()
        else :
            tl.setConstraint(ConstraintBound(c.name , self.m_v + self.m_k))

        self.m_hqpData.setData(priority, weight, tl.getConstraint())

        return 0 

        
    def addMotionTask(self, task, weight, priority, transition_time =0.0):
        assert weight >= 0.0
        assert transition_time >= 0.0

        tl = TaskLevel(task, priority)
        self.m_taskMotions.append(tl) # append class instance
        self.addTask(tl, weight, priority) # input task info into hqp data 

        return True            

    def addForceTask(self, task, weight, priority, transition_time = 0.0):
        assert weight >= 0.0
        assert transition_time >= 0.0

        tl = TaskLevel(task, priority)
        self.m_taskContactForces.append(tl)
        self.addTask(tl, weight, priority)

        return True

    def addTorqueTask(self, task, weight, priority, transition_time = 0.0):
        assert weight >= 0.0
        assert transition_time >= 0.0

        tl = TaskLevel(task, priority)
        self.m_taskActuations.append(tl)


        if priority > self.m_hqpData.size():
            for i in range(0, (priority-len(self.m_hqpData.m_HQPData))) :
                self.m_hqpData.m_HQPData.append([])

        c = tl.getTask().getConstraint()

        if c.isEquality():
            tl.setConstraint(ConstraintEquality(c.name, c.rows(), self.m_v + self.m_k))
            if priority == 0 and self.m_robot.isFloatingBase() :
                self.m_eq += c.rows()
        
        elif c.isInequality():
            tl.setConstraint(ConstraintInequality(c.name, c.rows(), self.m_v + self.m_k))        
            if priority == 0 and self.m_robot.isFloatingBase() :
                self.m_in += c.rows()            

        self.m_hqpData.setData(priority, weight, tl.getConstraint())

        return True

        
    def updateTaskWeight(self, task_name, weight):

        return False

    def addRigidContact(self, contact):
        cl = ContactLevel(contact)
        cl.m_index = self.m_k
        self.m_k += contact.n_force()
        self.m_contacts.append(cl) # append ContactLevel class

        self.resizeHqpData()

        motionConstr = contact.getMotionConstraint()
        cl.m_motionConstraint = ConstraintEquality(contact.name(), motionConstr.rows(), self.m_k + self.m_v)
        self.m_hqpData.setData(0, 1.0, cl.m_motionConstraint)

        forceConstr = contact.getForceConstraint()
        cl.m_forceConstraint = ConstraintInequality(contact.name(), forceConstr.rows(), self.m_k + self.m_v)
        self.m_hqpData.setData(0, 1.0, cl.m_forceConstraint)        

        forceRegConstr = contact.getForceRegularizationTask()
        cl.m_forceRegTask = ConstraintEquality(contact.name(), forceRegConstr.rows(), self.m_k + self.m_v)
        self.m_hqpData.setData(1, contact.getForceRegularizationWeight(), cl.m_forceRegTask)  

        self.m_eq += motionConstr.rows()
        self.m_in += forceConstr.rows()

        return True


    def removeTask(self, taskName, transition_time=0.0):
        return 0

    def removeRigidContact(self, contactName, transition_duration):
        if transition_duration > 0.0 :
            for i in range(0, len(self.m_contacts)): # contact level
                if self.m_contacts[i].getContact().m_name == contactName : # contact in contact level
                    transitionInfo = ContactTransitionInfo()
                    transitionInfo.m_contactLevel = self.m_contacts[i]
                    transitionInfo.m_time_start = self.m_t
                    transitionInfo.m_time_end = self.m_t + transition_duration
                    k = self.m_contacts[i].getContact().n_force()
                    
                    if len(self.m_f) >= self.m_contacts[i].m_index+k:
                        f = self.m_f[self.m_contacts[i].m_index : self.m_contacts[i].m_index+k]
                        transitionInfo.m_fMax_start = self.m_contacts[i].getContact().getNormalForce(f)
                    else : 
                        transitionInfo.m_fMax_start = self.m_contacts[i].getContact().getMaxNormalForce()

                    transitionInfo.m_fMax_end = self.m_contacts[i].getContact().getMinNormalForce() + 1e-3
                    self.m_contactTransitions.append(transitionInfo)
                return True
        return False

    def computeProblemData(self, time, q, v):
        self.m_t = time

        for i in range(0, len(self.m_contactTransitions)):
            c = self.m_contactTransitions[i]
            assert c.m_time_start <= self.m_t 
            if self.m_t  <= c.m_time_end:
                alpha = (self.m_t - c.m_time_start)/(c.m_time_end-c.m_time_start)
                fMax = c.m_fMax_start + alpha*(c.m_fMax_end-c.m_fMax_start)
                c.m_contactLevel.getContact().setMaxNormalForce(fMax) # m_contact > ContactBase
            else:
                self.removeRigidContact(c.m_contactLevel.getContact().name())
                del self.m_contactTransitions[i] # fix me?

        self.m_robot.computeAllTerms(self.m_data, q, v)

        for i in range(0, len(self.m_contacts)):
            cl = self.m_contacts[i] # get contactLevel class
            m = cl.getContact().n_force()
            mc = cl.getContact().computeMotionTask(time, q, v, self.m_data) # contact task > constraint
            cl.m_motionConstraint.setMatrix_middlecols(0,self.m_v,mc.matrix()) 
            cl.m_motionConstraint.setVector(mc.vector())  # no vector() in ConstraintBase?

            T = cl.getContact().getForceGeneratorMatrix()
            self.m_Jc[cl.m_index:cl.m_index+m,:] = T.transpose()*mc.matrix()

            fc = cl.getContact().computeForceTask(time, q, v, self.m_data)
            cl.m_forceConstraint.setMatrix_middlecols(self.m_v+cl.m_index, m , fc.matrix())
            cl.m_forceConstraint.setLowerBound(fc.lowerBound())
            cl.m_forceConstraint.setUpperBound(fc.upperBound())

            fr = cl.getContact().computeForceRegularizationTask(time, q, v, self.m_data)
            cl.m_forceRegTask.setMatrix_middlecols(self.m_v+cl.m_index, m, fr.matrix())
            cl.m_forceRegTask.setVector(fr.vector())

            for j in range(1, len(self.m_hqpData.m_HQPData)):
                if self.m_hqpData.m_HQPData[j][1].name() == cl.getContact().name(): # name of forceRegTask
                     self.m_hqpData.m_HQPData[j][0] = cl.getContact().getForceRegularizationWeight()
                break

        # u denote > centroidal dynamics : 6 dim
        # a denote > acutuation part
        if self.m_robot.isFloatingBase():
            M_a = self.m_robot.mass(self.m_data)[6:self.m_v,:]
            h_a = self.m_robot.nonLinearEffects(self.m_data)[6 :self.m_v]
            J_a = self.m_Jc[6:self.m_v,:]
            M_u = self.m_robot.mass(self.m_data)[0:6,:]
            h_u = self.m_robot.mass(self.m_data)[0:6]
            J_u = self.m_Jc[0:6,:]
        else: # fixed base
            M_a = self.m_robot.mass(self.m_data)
            h_a = self.m_robot.nonLinearEffects(self.m_data)
            J_a = self.m_Jc

        self.m_baseDynamics.setMatrix_middlecols(0, self.m_v-6, M_u)
        self.m_baseDynamics.setMatrix_middlecols(self.m_v, self.m_v+self.m_k, -J_u.transpose())
        self.m_baseDynamics.setVector(-h_u)

        for i in range(0, len(self.m_taskMotions)):
            c = self.m_taskMotions[i].getTask().compute(time, q, v)
            if c.isEquality():
                self.m_taskMotions[i].getConstraint().setMatrix(c.matrix())
                self.m_taskMotions[i].getConstraint().setVector(c.vector())
            elif c.isInequality():
                self.m_taskMotions[i].getConstraint().setMatrix(c.matrix())
                self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())
                self.m_taskMotions[i].getConstraint().setUpperBound(c.upperBound())
            else:
                self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())
                self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())

        # acuation part - skip it

        self.m_solutionDecoded = False
        return self.m_hqpData

    def decodeSolution(self, sol):
        if self.m_solutionDecoded == True:
            return True
        else :      
            if self.m_robot.isFloatingBase():
                M_a = self.m_robot.mass(self.m_data)[6:self.m_v,:]
                h_a = self.m_robot.nonLinearEffects(self.m_data)[6 :self.m_v]
                J_a = self.m_Jc[6:self.m_v,:]
            else: # fixed base
                M_a = self.m_robot.mass(self.m_data)
                h_a = self.m_robot.nonLinearEffects(self.m_data)
                J_a = self.m_Jc

        self.m_dv = sol.x[0:self.m_v]
        self.m_f = sol.x[self.m_v:self.m_v+self.m_k]
        self.m_tau = h_a
        self.m_tau += M_a*self.m_dv
        self.m_tau -= J_a.transpose()*self.m_f

        return True
        

    def getActuatorForces(self, sol):
        self.decodeSolution(sol)
        return self.m_tau

    def getAccelerations(self, sol):
        #return np.matrix(sol[0:self.m_robot.nv]).transpose()
        self.decodeSolution(sol)
        return self.m_dv

    def getContactForces(self, sol):
        self.decodeSolution(sol)
        return self.m_f

    def removeTask(self, task_name):
        return 0

    def remoteFromHqpData(self, name):
        return 0

              
