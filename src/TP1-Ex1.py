#!/usr/bin/env python3
import rospy
from src.controllers.Controller import ControlNode
from src.planners.TangentBug import TangentBug, TB_state
from src.utils.LidarScanner import LidarScanner

class TangentNode(ControlNode):
    def __init__(self,params):
        super().__init__(params)
        self.robot = TangentBug
        self.lidar = LidarScanner('/base_scan')
    
    def iteration(self):
        pass



if __name__ == "__main__":
    try:
        params = {"x_goal":0,
                  "y_goal":0}
        node = TangentNode(params)
    except rospy.ROSInterruptException:
        pass