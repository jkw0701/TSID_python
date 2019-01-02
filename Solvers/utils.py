from pinocchio import se3 #SE3 ??
import copy
import numpy as np

def printHQPData(HQPData, verbose=False):
    priority = 0
    for i in range(0, HQPData.size()):
        if HQPData.data()[i]:
            print "Level #", priority, ":"
            for j in range(0, len(HQPData.data()[i])):
                c =  HQPData.data()[i][j][1]
                if c.isEquality():
                    print "   Equality Task Name:", c.name, "Weight:", HQPData.data()[i][j][0], "Matrix Size: ", c.rows(), "X", c.cols()
                elif c.isInequality():
                    print "   Inequality Task Name:", c.name, "Weight:", HQPData.data()[i][j][0]
                else:
                    print "   Bound Task Name:", c.name, "Weight:", HQPData.data()[i][j][0]
            priority += 1

    if verbose:
        priority = 0
        for i in range(0, HQPData.size()):
            if HQPData.data()[i]:
                print "Level #", priority, ":"
                for j in range(0, len(HQPData.data()[i])):
                    c = HQPData.data()[i][j][1]
                    if c.isEquality():
                        print "   Equality Task Name:", c.name
                        print "   A:", c.matrix()
                        print "   b", c.vector().transpose()
                    elif c.isInequality():
                        print "   Inequality Task Name:", c.name
                        print "   A:", c.matrix()
                        print "   lb", c.lowerBound().transpose()
                        print "   ub", c.upperBound().transpose()
                    else:
                        print "   lb", c.lowerBound().transpose()
                        print "   ub", c.upperBound().transpose()
                priority += 1
