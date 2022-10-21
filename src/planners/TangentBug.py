from enum import Enum

import rospy
from rosgraph_msgs.msg import Clock

from planners.PotentialField import PotentialField
from utils.potential_field import potential_field
from utils.LidarScanner import LidarScanner
from utils.utils import *
import numpy as np

class TangentBug(PotentialField):
    class TB_state(Enum):
        STOP = 0
        FOLLOW_GOAL = 1
        FOLLOW_Oi = 2
        FOLLOW_TANG = 3

    def __init__(self, ksi=1, d_sat=1.5, eta=1/2, d_act=2):
        super().__init__(ksi, d_sat, eta, d_act)
        self.state = self.TB_state.FOLLOW_GOAL
        rospy.init_node('control_node')
        rospy.Subscriber('/clock', Clock, self.callback_time)

        self.oi_dist = None
        self.d_followed = np.inf
        self.close_by = False
        self.time_start = None

        self.state_func_dict = {
            self.TB_state.FOLLOW_GOAL: self.follow_goal,
            self.TB_state.FOLLOW_Oi: self.follow_Oi,
            self.TB_state.FOLLOW_TANG: self.follow_tangent,
        }
        print("here")

    def callback_time(self,data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6

    def reset(self):
        self.state = self.TB_state.FOLLOW_GOAL
        self.d_followed = np.inf

    # Useful methods

    def obj_in_path(self,goal,position,angle):
        theta = relative_angle(position,goal,angle)
        direct = self.scanner.get_dist(theta) != self.scanner.rmax
        close_by = (self.scanner.get_front(0.1)[1] < self.scanner.rmax).any()
        return direct or close_by

    def min_reach(self,goal,position,angle):
        ranges = self.scanner.ranges
        points = [(a,r,get_measured_points(*position,a+angle,r)) for a,r in ranges if r != self.scanner.rmax]
        dist = [(a,r,vec_norm(p,goal)[1]) for a,r,p in points]
        return min(dist, key = lambda x: x[2])

    def oi_heuristic(self, position, goal,theta):
        x, y = position
        oi_list = self.scanner.discontinuities()
        oi_list = [a + theta for a in oi_list]
        oi_list = [get_measured_points(x, y, theta, self.scanner.rmax) for theta in oi_list]
        dist_list = [vec_norm(goal, oi)[1] for oi in oi_list]
        return oi_list[idxmin(dist_list)]

    # Tangent state end condition

    def check_no_path(self,position,angle):
        ranges = self.scanner.ranges
        points = [get_measured_points(*position,a+angle,r) for a,r in ranges]
        dist = np.array([segment_dist(points[i-1],p,self.q_followed) for i,p in enumerate(points)])
        close_by = (dist <= 0.2).any()
        if (not self.close_by and close_by) or self.time_start+1e7 < self.time:
            return False
        if (self.close_by and not close_by) and self.time_start+1e4 < self.time:
            self.close_by = close_by
        return True

    # State methods

    def follow_goal(self, goal, position, angle):
        q_wall = self.get_q_wall(position,angle)
        U = potential_field(
            position, goal, self.ksi, self.d_sat, q_wall, self.eta/10, self.d_act
        )
        return U

    def follow_Oi(self, _, position, angle):
        q_wall = self.get_q_wall(position,angle)
        U = potential_field(
            position, self.q_oi, self.ksi, self.d_sat, q_wall, self.eta, self.d_act
        )
        return U

    def follow_tangent(self, _, position,__):
        D, dnorm = vec_norm(position,self.q_wall)
        N, norm = vec_norm(D,0.75*D/dnorm)
        if norm == 0:
            norm = 1e-3
        T = self.ccw * ortogonal_vec(D) / dnorm
        G = - norm / np.sqrt(1 + norm**2)
        H = 1/np.sqrt(1+norm**2)
        U = self.ksi*self.d_sat*(G*N+H*T)
        return U

    # Compute next U

    def get_next(self, goal, position, angle):
        if vec_norm(goal,position)[1] <= 0.2:
            rospy.loginfo("Goal reached")
            self.state = self.TB_state.STOP

        if self.state == self.TB_state.STOP:
            return None

        if self.state == self.TB_state.FOLLOW_TANG:
            if not self.check_no_path(position,angle):
                rospy.loginfo("No path to goal")
                self.state = self.TB_state.STOP
                return None

            self.q_wall = self.get_q_wall(position,angle)
            theta,r_reach,d_reach = self.min_reach(goal,position,angle)
            if d_reach < self.d_followed:
                self.d_followed = d_reach
                self.q_followed = get_measured_points(*position,theta+angle,r_reach)
                self.time_start = self.time
                self.close_by = True
                if not self.obj_in_path(goal,position,angle):
                    self.state = self.TB_state.FOLLOW_GOAL
        elif not self.obj_in_path(goal,position,angle):
            self.state = self.TB_state.FOLLOW_GOAL
            self.oi_dist = None
        elif self.state == self.TB_state.FOLLOW_GOAL:
            self.state = self.TB_state.FOLLOW_Oi

        if self.state == self.TB_state.FOLLOW_Oi:
            q_oi = self.oi_heuristic(position, goal,angle)
            self.q_wall = self.get_q_wall(position,angle)
            oi_dist = vec_norm(q_oi,goal)[1]
            if self.oi_dist is None or oi_dist <= self.oi_dist + 0.2:
                self.oi_dist = oi_dist
                self.q_oi = q_oi
            else:
                self.state = self.TB_state.FOLLOW_TANG
                theta,r_reach,d_followed = self.min_reach(goal,position,angle)
                if d_followed < self.d_followed:
                    self.d_followed = d_followed
                    self.q_followed = get_measured_points(*position,theta+angle,r_reach)
                self.time_start = self.time
                self.close_by = True
                self.ccw = ccw(position,q_oi,goal)

        return self.state_func_dict[self.state](goal,position,angle)
