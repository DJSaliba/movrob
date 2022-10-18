#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from src.planners.PotentialField import PotentialField

class PotentialNode(ControlNode):
    def __init__(self):
        self.planner = PotentialField()
        super().__init__()

    def setup(self):
        while not self.lidar.ranges:
            self.rate.sleep()

    def plan(self):
        self.U = self.planner.get_next(self.goal,[self.x,self.y],self.theta)

if __name__ == "__main__":
    try:
        node = PotentialNode()
        node.run()
    except rospy.ROSInterruptException:
        pass