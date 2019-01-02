from cvxopt import matrix, solvers
import numpy as np
import copy
from fwd import HQPData
import conf
import qpoases

np.set_printoptions(linewidth=400, suppress=True, threshold=np.nan)

class HQPSolver(object):
    def __init__(self, name):
        self.m_name = name
        self.m_hessian_regularization = 1e-5
        self.m_n = 0
        self.m_neq = 0
        self.m_nin = 0
        self.m_bound = 0

    def solve(self, HQPdata):
        assert HQPdata.data() > 0
        problemData = self.SqueezeData(HQPdata)
        cl0 = problemData[0]
        n = cl0[0][1].cols()

        neq = np.matrix(np.zeros(len(problemData))).transpose()
        nin = np.matrix(np.zeros(len(problemData))).transpose()
        nbound = np.matrix(np.zeros(len(problemData))).transpose()
        slack_opt = []
        xOpt = []

        for i in range(0, len(problemData)):
            cl = problemData[i]
            for j in range(0, len(cl)):
                assert n == cl[j][1].cols()
                if cl[j][1].isEquality():
                    neq[i] += cl[j][1].rows()
                elif cl[j][1].isInequality():
                    nin[i] += cl[j][1].rows()
                else:
                    nbound[i] += cl[j][1].rows()

        self.resize(n, neq, nin, nbound)

        i_eq = 0
        i_in = 0
        i_bound = 0

        for i in range(0, len(problemData)):
            cl = problemData[i]
            for j in range(0, len(cl)):
                if cl[j][1].isEquality():
                    self.A[i][i_eq: i_eq + cl[j][1].rows(), 0:n] = copy.deepcopy(cl[j][1].matrix())
                    self.A[i][i_eq: i_eq + cl[j][1].rows(), n:n +cl[j][1].rows()] = np.matrix(np.eye(cl[j][1].rows(), cl[j][1].rows()))
                    self.Alb[i][i_eq: i_eq + cl[j][1].rows()] = copy.deepcopy(cl[j][1].vector())
                    self.Aub[i][i_eq: i_eq + cl[j][1].rows()] = copy.deepcopy(cl[j][1].vector())

                    i_eq += cl[j][1].rows()

        for i in range(1, len(problemData)):
            if not len(self.A[i]) == len(self.A[i-1]):
                self.A[i][0:len(self.A[i-1]), 0:n] = copy.deepcopy(self.A[i-1][:, 0:n])
                self.Alb[i][0:len(self.Alb[i-1])] = copy.deepcopy(self.Alb[i-1])
                self.Aub[i][0:len(self.Aub[i-1])] = copy.deepcopy(self.Aub[i-1])

        for i in range(0, len(problemData)):
            if i == 0 :
                n_size = n + len(self.A[0])
                c_size = len(self.A[0])
            else:
                c_size = len(self.A[i])
                n_size = n + len(self.A[i])- len(self.A[i-1])

            H = np.matrix(np.eye(n_size, n_size))
            H[:n, :n] = np.matrix(np.eye(n, n)) * self.m_hessian_regularization
            g = np.array(np.zeros(n_size))

            lb = np.hstack([conf.JOINT_ACC_LB, -1000000 * np.array(np.ones(c_size))])
            ub = np.hstack([conf.JOINT_ACC_UB, 1000000 * np.array(np.ones(c_size))])
            G_ieq = np.vstack ([-1.0 * np.matrix(np.eye(n_size, n_size)), np.matrix(np.eye(n_size, n_size))])
            g_ieq = np.hstack ([-1.0 * lb, ub])

            #print slack_opt, "sl"
            #print self.Alb[i][:len(self.Alb[i]) - c_size], "alb"


            if i is not 0:
                for k in range(0, len(slack_opt)):
                    self.Alb[i][k, 0] -= slack_opt[k]

            #from qpoases import PySQProblem, PyOptions, PyPrintLevel # 왜 다를까???
            option = qpoases.PyOptions()
            qp = qpoases.PySQProblem(n_size, c_size)
            option.printLevel = qpoases.PyPrintLevel.NONE
            qp.setOptions(option)

            A_tmp = np.squeeze(np.asarray(self.A[i]))
            qp.init(H, g, A_tmp.reshape((c_size, n_size)).transpose(), np.squeeze(np.asarray(lb)), np.squeeze(np.asarray(ub)), np.squeeze(np.asarray(self.Alb[i].transpose())), np.squeeze(np.asarray(self.Alb[i].transpose())), np.array([100]))
            xqp = np.zeros(n_size)
            qp.getPrimalSolution(xqp)

            #sol = solvers.qp(matrix(H, tc='d'), matrix(g, tc='d'), matrix(G_ieq, tc='d'), matrix(g_ieq, tc='d'), matrix(self.A[i], tc='d'), matrix(self.Alb[i], tc='d'), options={'show_progress': False})
            #xOpt = np.matrix(sol['x']).transpose()
            xOpt = np.squeeze(np.asarray(xOpt))
            slack_opt.extend(xqp[n:])
            #print xOpt, "cvx"

        return xqp

    def resize(self, n, neq, nin, nbound):
        self.A = []
        self.Alb = []
        self.Aub = []
        self.lb = []
        self.ub = []

        self.slack_num = []

        for i in range(0, len(neq)):
            slack = 0
            for j in range(0, i):
                slack += self.slack_num[j]

            if neq[i] + nin[i] > 0:
                self.A.append(np.matrix(np.zeros((int(neq[i] + nin[i] + slack), n + int(neq[i] + nin[i])))))
                self.Alb.append(np.matrix(np.zeros(int(neq[i] + nin[i] + slack))).transpose())
                self.Aub.append(np.matrix(np.zeros(int(neq[i] + nin[i] + slack))).transpose())
            else:
                self.lb.append(np.matrix(np.zeros(int(nbound[i] + slack))).transpose())
                self.ub.append(np.matrix(np.zeros(int(nbound[i] + slack))).transpose())
            self.slack_num.append(int(neq[i] + nin[i] + nbound[i]))

        return True

    def SqueezeData(self, HQPdata):
        j = 0
        for i in range(0, HQPdata.size()):
            if HQPdata.data()[i]:
                j += 1

        ProblemData = HQPData()
        ProblemData.resize(j)

        j = 0
        for i in range(0, HQPdata.size()):
            if HQPdata.data()[i]:
                ProblemData.data()[j] = copy.deepcopy(HQPdata.data()[i])
                j += 1

        return ProblemData.data()