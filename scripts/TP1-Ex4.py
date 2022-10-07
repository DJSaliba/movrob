import rospy
from src.Controller import ControlNode
from src.WaveFront import WaveFront

class WaveNode(ControlNode):
    def __init__(self,params,image_path):
        super().__init__(params)
        self.planner = WaveFront(image_path)
    
    def goal_update(self):
        self.planner(self.goal)

    def start(self):
        self.goal_update()

    def iteration(self):
        self.U = self.planner.get_next()
        if self.U is None or self.U == (0,0):
            return
        self.vel = self.controller.feedback_linearization(self.U,self.theta)
        self.publish_vel()


if __name__ == "__main__":
    try:
        params = {"x_goal":0,
                  "y_goal":0}
        node = WaveNode(params)
    except rospy.ROSInterruptException:
        pass