import numpy as np 
import copy
from task_base import *

class TaskMotion(TaskBase):
    def __init__(self, name, robot):
        TaskBase.__init__(self, name, robot)

    def getReference(self):
        return 0

    def getDesiredAccleration(self):
        return 0

    def getAcceleration(self):
        return 0

    def position_error(self):
        return 0

    def velocity_error(self):
        return 0

    def position(self):
        return 0

    def position_ref(self):
        return 0

    def velocity(self):
        return 0

    def velocity_ref(self):
        return 0    