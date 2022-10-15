#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.TangentBug import TangentBug
from utils.LidarScanner import LidarScanner

class TangentNode(ControlNode):
    def __init__(self):
        super().__init__()
        self.planner = TangentBug()
        self.lidar = LidarScanner('/base_scan')
    
    def start(self):
        while not self.lidar.ranges:
            self.rate.sleep()

    def iteration(self):
        self.U = self.planner.plan(self.goal,[self.x,self.y],self.theta)
        if self.U is None:
            self.rate.sleep()
            return
        self.vel = self.controller.feedback_linearization(self.U,self.theta)
        self.publish_vel()

if __name__ == "__main__":
    try:
        node = TangentNode()
        node.run()
    except rospy.ROSInterruptException:
        pass