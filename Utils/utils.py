from pinocchio import se3
import copy
import numpy as np

def vectorToSE3(vec):   
    M = se3.SE3()

    rot_tmp = np.matrix(np.ones((3, 3)))
    rot_tmp[0:3, 0] = vec[3:6]
    rot_tmp[0:3, 1] = vec[6:9]
    rot_tmp[0:3, 2] = vec[9:12]

    M.translation = copy.deepcopy(vec[0:3])
    M.rotation= rot_tmp
    return M

def SE3toVector(M):
    ref = np.matrix(np.zeros(12)).transpose()
    ref[0:3] = copy.deepcopy(M.translation)
    ref[3:6] = copy.deepcopy(M.rotation[0:3, 0])
    ref[6:9] = copy.deepcopy(M.rotation[0:3, 1])
    ref[9:12] = copy.deepcopy(M.rotation[0:3, 2])

    return ref


def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, x[0]],
                     [x[1], x[0], 0]])