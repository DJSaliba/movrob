#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.QLearning import QLearning

class QLNode(ControlNode):
    def __init__(self,image_path):
        self.planner = QLearning(image_path)
        super().__init__()
    
    def goal_update(self):
        self.planner(self.goal,(self.x,self.y))

    def start(self):
        self.goal_update()

    def plan(self):
        self.U = self.planner.get_next((self.x,self.y))

if __name__ == "__main__":
    try:
        image_path = "/home/djsaliba/ROS_WS/src/movrob/worlds/test_wavefront.png"
        node = QLNode(image_path)
        node.run()
    except rospy.ROSInterruptException:
        pass