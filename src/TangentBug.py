import rospy
import enum
from geometry_msgs.msg import Pose
from enum import Enum
from src.utils.potential_field import atractive_field

class TB_state(Enum):
    REACHED_GOAL = 0
    FOLLOW_GOAL = 1
    FOLLOW_Oi = 2
    FOLLOW_TANG = 3

class TangentBug():
    def __init__(self,ksi,d_sat,eta,d_act):
        self.state = TB_state.FOLLOW_GOAL
        self.U = (0,0)
        rospy.Subscriber("/goal",Pose,self.callback_goal)
        self.ksi

    def follow_goal(self, position):
        return atractive_field(position,self.goal,self.ksi,self.d_sat)
    
    def follow_Oi(self, position,):
        pass