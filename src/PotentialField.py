import rospy

from src.LidarScanner import LidarScanner

class PotentialField():
    def __init__(self, ksi, d_sat, eta, d_act):
        self.U = (0, 0)
        self.scanner = LidarScanner("/base_scan")

        self.ksi = ksi
        self.d_sat = d_sat
        self.eta = eta
        self.d_act = d_act