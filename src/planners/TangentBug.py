from enum import Enum

import rospy
from rosgraph_msgs.msg import Clock

from utils.potential_field import potential_field
from utils.LidarScanner import LidarScanner
from utils.utils import vec_norm, get_measured_points, idxmin, relative_angle, ccw, ortogonal_vec, vec_angle
import numpy as np

class TB_state(Enum):
    STOP = 0
    FOLLOW_GOAL = 1
    FOLLOW_Oi = 2
    FOLLOW_TANG = 3


class TangentBug():
    def __init__(self, ksi=1, d_sat=1.5, eta=1/2, d_act=2):
        self.state = TB_state.FOLLOW_GOAL
        self.U = (0, 0)
        rospy.init_node('control_node')
        self.scanner = LidarScanner("/base_scan")
        rospy.Subscriber('/clock', Clock, self.callback_time)

        self.oi_dist = None
        self.q_oi_follow = None
        self.time_start = None

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

    def callback_time(self,data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6

    def stop(self, *args):
        return None

    def obj_in_path(self,goal,position,angle):
        theta = relative_angle(position,goal,angle)
        direct = self.scanner.get_dist(theta) != self.scanner.rmax
        close_by = (self.scanner.get_front(0.1)[1] <= 1).any()
        return direct or close_by

    def follow_goal(self, goal, position, angle):
        q_closest = self.get_q_closest(position,angle)
        U = potential_field(
            position, goal, self.ksi, self.d_sat, q_closest, self.eta, self.d_act
        )
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
        D, dnorm = vec_norm(position,self.q_closest)
        E, enorm = vec_norm(D,D/dnorm)
        if enorm == 0:
            enorm = 1e-3
        #E = D - 2*self.p0*D/(np.linalg.norm(D)+1e-6)
        #RD = np.array([-D[1], D[0], 0.0])
        #F = self.d*self.c*(G*E/(np.linalg.norm(E)+1e-6) + H*RD/(np.linalg.norm(RD)))
        T = self.ccw * ortogonal_vec(D) / dnorm
        #G = -2*np.atan(np.linalg.norm(E))/np.pi
        G = - enorm / np.sqrt(1 + enorm**2)
        H = 1/ np.sqrt(1+enorm**2)
        #H = self.follow_dir*np.sqrt(1-G**2+1e-6)
        return self.ksi*self.eta*(G*E+H*T)

    def check_no_path(self):
        #print(vec_norm(self.q_init_tang,self.q_closest)[1],self.q_init_tang)
        if vec_norm(self.q_init_tang,self.q_closest)[1] <=1:
            if not self.just_started:
                self.state = TB_state.STOP
                return False
        elif self.time_start > self.time + 2e3:
            self.just_started = False
        return True

    def plan(self, goal, position, angle):
        q_oi = None
        print(self.state)
        if self.state == TB_state.FOLLOW_TANG:
            if not self.check_no_path():
                rospy.loginfo("No path to goal")
                self.state = TB_state.STOP
            else:
                self.q_closest = self.get_q_closest(position,angle)
                front = self.scanner.get_front(self.scanner.increment)[0][0]
                if vec_norm(self.q_closest,front)[1] < 0.1:
                    self.q_closest = front
                self.d_closest = vec_norm(self.q_closest,goal)[1]

                theta = relative_angle(position,goal,angle)
                d_goal = vec_norm(position,goal)[1]
                print(self.d_init_tang > self.d_closest,vec_angle(position,self.q_closest,goal),theta,self.scanner.get_dist(theta))
                if (self.d_closest < self.d_init_tang and
                    vec_angle(position,self.q_closest,goal) > np.pi/2 and
                    self.scanner.get_dist(theta) >= min(self.scanner.rmax,d_goal)):
                    self.state = TB_state.FOLLOW_GOAL
        elif self.obj_in_path(goal,position,angle):
            if self.state == TB_state.FOLLOW_GOAL:
                self.state = TB_state.FOLLOW_Oi
        else:
            self.state = TB_state.FOLLOW_GOAL
            self.oi_dist = None
            self.q_oi = None
            self.q_init_tang = None
            self.q_closest = None

        if self.state == TB_state.FOLLOW_Oi:
            q_oi = self.oi_heuristic(position, goal,angle)
            self.q_closest = self.get_q_closest(position,angle)
            oi_dist = vec_norm(q_oi,goal)[1]
            if self.oi_dist is None or oi_dist <= self.oi_dist + 0.2:
                self.oi_dist = oi_dist
                self.q_oi = q_oi
            else:
#                theta = relative_angle(position,goal,angle)
 #               if self.scanner.get_dist(theta) < 2:
                self.state = TB_state.FOLLOW_TANG
                self.q_init_tang = self.get_q_closest(position,angle)
                self.d_init_tang = vec_norm(self.q_init_tang,goal)[1]
                self.just_started = True
                self.time_start = self.time
                self.ccw = ccw(position,q_oi,goal)
  #              else:
  #                  return self.follow_goal(q_oi,position,angle)

        if vec_norm(goal,position)[1] <= 0.2:
            rospy.loginfo("Goal reached")
            self.state = TB_state.STOP

        return self.state_func_dict[self.state](goal,position,angle)
