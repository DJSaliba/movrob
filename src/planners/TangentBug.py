from enum import Enum

import rospy
from geometry_msgs.msg import Pose

from src.utils.potential_field import atractive_field, potential_field
from src.LidarScanner import LidarScanner
from src.utils.utils import euc_dist, get_measured_points, idxmin


class TB_state(Enum):
    REACHED_GOAL = 0
    FOLLOW_GOAL = 1
    FOLLOW_Oi = 2
    FOLLOW_TANG = 3


class TangentBug:
    def __init__(self, ksi, d_sat, eta, d_act):
        self.state = TB_state.FOLLOW_GOAL
        self.U = (0, 0)
        self.scanner = LidarScanner("/base_scan")
        self.max_dist = self.scanner.range_max

        self.ksi = ksi
        self.d_sat = d_sat
        self.eta = eta
        self.d_act = d_act

        self.state_func_dict = {
            TB_state.REACHED_GOAL: self.stop,
            TB_state.FOLLOW_GOAL: self.follow_goal,
            TB_state.FOLLOW_Oi: self.follow_Oi,
            TB_state.FOLLOW_TANG: self.follow_tangent,
        }

    def stop(self, _):
        self.U = None

    def follow_goal(self, position, goal):
        U = atractive_field(position, self.goal, self.ksi, self.d_sat)
        return U

    def oi_heuristic(self, position, goal):
        x, y, theta = position
        oi_list = self.scanner.discontinuities()
        oi_list = [a + theta for a in oi_list]
        oi_list = [get_measured_points(x, y, theta, self.max_dist) for theta in oi_list]
        dist_list = [euc_dist(goal, oi)[1] for oi in oi_list]
        return oi_list[idxmin(dist_list)]

    def follow_Oi(self, position, goal):
        q_closest = self.scanner.get_closest()
        q_oi = self.oi_heuristic(position, goal)
        U = potential_field(
            position, q_oi, self.ksi, self.d_sat, q_closest, self.eta, self.d_act
        )
        return U

    def follow_tangent(self, position, goal):
        pass

    def plan(self, position, goal):

        return self.state_dict[self.state](position, goal)
