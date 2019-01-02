class ConstraintLevel(object):
    def __init__(self):
        self.m_constraintLevel = []

    def pushback(self, ConstraintBase):
        self.m_constraintLevel.append(ConstraintBase)

class HQPData(object):
    def __init__(self):
        self.m_HQPData = []
        
    def pushback(self):
        self.m_HQPData.append([]) # list

    def data(self):
        return self.m_HQPData

    # resize of hqpData ???
    def resize(self, number):
        for i in range(0, number):
            self.pushback() # pushback what??

    def size(self):
        return len(self.m_HQPData)

    def setData(self, number, weight, Constraint):
        assert number < self.size()
        self.m_HQPData[number].append((weight, Constraint)) #list can append any kind of data set

    def showState(self):
        print self.m_HQPData