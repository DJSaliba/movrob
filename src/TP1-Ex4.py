#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.WaveFront import WaveFront

class WaveNode(ControlNode):
    def __init__(self,params,image_path):
        self.planner = WaveFront(image_path)
        super().__init__(params)
    
    def goal_update(self):
        self.planner(self.goal)

    def start(self):
        self.goal_update()

    def iteration(self):
        self.U = self.planner.get_next(self.x,self.y)
        if self.U is None or self.U == (0,0):
            self.rate.sleep()
            return
        self.vel = self.controller.feedback_linearization(self.U,self.theta)
        self.publish_vel()


if __name__ == "__main__":
    try:
        params = {"x_goal":0,
                  "y_goal":0}
        image_path = "/workspaces/src/movrob/worlds/test_env.png"
        node = WaveNode(params,image_path)
        node.run()
    except rospy.ROSInterruptException:
        pass