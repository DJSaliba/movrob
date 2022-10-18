#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from src.planners.PathFollower import PathFollower
import numpy as np

class PathNode(ControlNode):
    def __init__(self,path):
        self.planner = PathFollower(path)
        super().__init__()

    def plan(self):
        self.U = self.planner.get_next(self.goal,[self.x,self.y],self.theta)

if __name__ == "__main__":
    try:
        path = lambda t: (np.cos(t),np.sin(t))
        node = PathNode(path)
        node.run()
    except rospy.ROSInterruptException:
        pass