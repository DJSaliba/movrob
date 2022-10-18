#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.TangentBug import TangentBug

class TangentNode(ControlNode):
    def __init__(self):
        self.planner = TangentBug()
        super().__init__()
    
    def goal_update(self):
        self.planner.reset()

    def setup(self):
        while not self.planner.scanner.ranges:
            self.rate.sleep()

    def plan(self):
        self.U = self.planner.get_next(self.goal,[self.x,self.y],self.theta)

if __name__ == "__main__":
    try:
        node = TangentNode()
        node.run()
    except rospy.ROSInterruptException:
        pass