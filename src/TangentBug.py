import rospy
import enum

from enum import Enum

class TB_state(Enum):
    REACHED_GOAL = 0
    FOLLOW_GOAL = 1
    FOLLOW_Oi = 2
    FOLLOW_TANG = 3

class TangentBug():
    def __init__(self):
        self.state = TB_state.FOLLOW_GOAL