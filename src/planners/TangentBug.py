from enum import Enum

import rospy
from geometry_msgs.msg import Pose

from utils.potential_field import atractive_field, potential_field
from utils.LidarScanner import LidarScanner
from utils.utils import vec_norm, get_measured_points, idxmin, vec_angle


class TB_state(Enum):
    STOP = 0
    FOLLOW_GOAL = 1
    FOLLOW_Oi = 2
    FOLLOW_TANG = 3


class TangentBug():
    def __init__(self, ksi=1, d_sat=0.75, eta=2, d_act=2):
        self.state = TB_state.FOLLOW_GOAL
        self.U = (0, 0)
        rospy.init_node('control_node')
        self.scanner = LidarScanner("/base_scan")

        self.oi_dist = None
        self.q_oi_follow = None

        self.ksi = ksi
        self.d_sat = d_sat
        self.eta = eta
        self.d_act = d_act

        self.state_func_dict = {
            TB_state.STOP: self.stop,
            TB_state.FOLLOW_GOAL: self.follow_goal,
            TB_state.FOLLOW_Oi: self.follow_Oi,
            TB_state.FOLLOW_TANG: self.follow_tangent,
        }

    def stop(self, *args):
        rospy.loginfo("Goal reached")
        return None

    def obj_in_path(self,goal,position,angle):
        theta = vec_angle(position,goal,angle)
        print(theta,self.scanner.get_dist(theta))
        return self.scanner.get_dist(theta) != self.scanner.rmax

    def follow_goal(self, goal, position, _):
        U = atractive_field(position,goal, self.ksi, self.d_sat)
        return U

    def oi_heuristic(self, position, goal,theta):
        x, y = position
        oi_list = self.scanner.discontinuities()
        oi_list = [a + theta for a in oi_list]
        oi_list = [get_measured_points(x, y, theta, self.scanner.rmax) for theta in oi_list]
        dist_list = [vec_norm(goal, oi)[1] for oi in oi_list]
        return oi_list[idxmin(dist_list)]

    def get_q_closest(self,position,theta):
        x,y = position
        idx = self.scanner.get_closest()
        a,r = self.scanner.ranges[idx]
        return get_measured_points(x,y,a+theta,r)

    def follow_Oi(self, goal, position, angle):
        q_closest = self.get_q_closest(position,angle)
        U = potential_field(
            position, self.q_oi, self.ksi, self.d_sat, q_closest, self.eta, self.d_act
        )
        return U

    def follow_tangent(self, goal, position,_):
        raise NotImplemented
        pass

    def plan(self, goal, position, angle):
        q_oi = None
        if self.obj_in_path(goal,position,angle):
            if self.state == TB_state.FOLLOW_GOAL:
                self.state = TB_state.FOLLOW_Oi
        else:
            self.state = TB_state.FOLLOW_GOAL
            self.oi_dist = None
            self.q_oi = None

        if self.state == TB_state.FOLLOW_Oi:
            q_oi = self.oi_heuristic(position, goal,angle)
            oi_dist = vec_norm(q_oi,goal)[1]
            if self.oi_dist is None or oi_dist <= self.oi_dist :
                self.oi_dist = oi_dist
                self.q_oi = q_oi
            else:
                self.state == TB_state.FOLLOW_TANG

        if vec_norm(goal,position)[1] <= 0.2:
            self.state = TB_state.STOP

        return self.state_func_dict[self.state](goal, position,angle)
