#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.Astar import Astar

class AstarNode(ControlNode):
    def __init__(self,image_path):
        self.planner = Astar(image_path)
        super().__init__()
    
    def goal_update(self):
        self.planner(self.goal,(self.x,self.y))

    def start(self):
        self.goal_update()

    def plan(self):
        self.U = self.planner.get_next(self.x,self.y)
ty
if __name__ == "__main__":
    try:
        image_path = "/workspaces/src/movrob/worlds/test_wavefront.png"
        node = Astar(image_path)
        node.run()
    except rospy.ROSInterruptException:
        pass