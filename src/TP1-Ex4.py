#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.WaveFront import WaveFront

class WaveNode(ControlNode):
    def __init__(self,image_path):
        self.planner = WaveFront(image_path)
        super().__init__()
    
    def goal_update(self):
        self.planner(self.goal)

    def start(self):
        self.goal_update()

    def plan(self):
        self.U = self.planner.get_next(self.x,self.y)

if __name__ == "__main__":
    try:
        image_path = "/workspaces/src/movrob/worlds/test_wavefront.png"
        node = WaveNode(image_path)
        node.run()
    except rospy.ROSInterruptException:
        pass