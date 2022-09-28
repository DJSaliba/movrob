import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarScanner():
    def __init__(self,scan_topic):
        # Scanned area
        self.ang_min = None
        self.ang_max = None
        self.increment = None
        # Scan results
        self.rmin = None
        self.rmax = None
        self.ranges = []
        
        rospy.Subscriber(scan_topic, LaserScan, self.callback_scan)

    def callback_scan(self,data):
        # Scanned area
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.increment = data.angle_increment
        # Scan results
        self.range_min = data.range_min
        self.range_max = data.range_max
        self.ranges = [(a,r) for a,r in zip(np.arange(self.ang_min, self.ang_max,
                                                      self.increment), data.ranges)]
    
    def discontinuities(self):
        for i, (a,r) in enumerate(self.ranges):
            disc = []
            if ((self.ranges[i-1][1] == self.range_max and a != self.range_max) or
                (a == self.range_max and self.ranges[i-1][1] != self.range_max)):
                disc.append(a)
        return disc
    
    def get_closest(self):
        return min(self.ranges,key = lambda x: x[1])
    
    def get_segment_from_angle(angle):
        pass