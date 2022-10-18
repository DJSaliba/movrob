import rospy

from utils.LidarScanner import LidarScanner
from utils.potential_field import potential_field
from utils.utils import get_measured_points
from planners.Planner import Planner

class PotentialField(Planner):
    def __init__(self, ksi=1, d_sat=1.5, eta=1/2, d_act=2):
        self.scanner = LidarScanner("/base_scan")
        self.ksi = ksi
        self.d_sat = d_sat
        self.eta = eta
        self.d_act = d_act

    def get_q_wall(self,position,theta):
        x,y = position
        idx = self.scanner.get_closest()
        a,r = self.scanner.ranges[idx]
        return get_measured_points(x,y,a+theta,r)

    def get_next(self, goal, position, angle):
        q_wall = self.get_q_wall(position,angle)
        U = potential_field(
            position, goal, self.ksi, self.d_sat, q_wall, self.eta, self.d_act
        )
        return U
