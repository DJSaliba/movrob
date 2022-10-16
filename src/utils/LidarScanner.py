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
        self.ang_min = data.angle_min
        self.ang_max = data.angle_max
        self.increment = data.angle_increment
        # Scan results
        self.rmin = data.range_min
        self.rmax = data.range_max
        self.ranges = [(a,r) for a,r in zip(np.arange(self.ang_min, self.ang_max,
                                                      self.increment), data.ranges)]

    def continuities(self):
        cont_start = []
        cont_end = []
        ranges = self.ranges.copy()
        for i, (a,r) in enumerate(ranges):
            prev = ranges[i-1][1]
            next = ranges[(i+1)%len(ranges)][1]
            if r != self.rmax:
                if prev == self.rmax or abs(r-prev) > self.rmax/2:
                    cont_start.append(a)
                if next == self.rmax or abs(next-r) > self.rmax/2:
                    cont_end.append(a)
        if cont_start[-1]> cont_end[-1]:
            cont_end.append(cont_end.pop(0))
        return list(zip(cont_start,cont_end))

    def discontinuities(self):
        cont_start,cont_end = zip(*self.continuities())
        disc = cont_start + cont_end
        return np.sort(disc)

    def get_dist(self,angle):
        angles, _ = zip(*self.ranges)
        angle_diff = np.absolute(np.array(angles)-angle)
        idx = np.argmin(angle_diff)
        return self.ranges[idx][1]

    def get_closest(self):
        return np.argmin(list(zip(*self.ranges))[1])

    def get_front(self,angle):
        a = [a for a,_ in self.ranges if np.abs(a)<=angle]
        r = [r for a,r in self.ranges if np.abs(a)<=angle]
        return np.array(a),np.array(r)
#    def get_segment_from_angle(angle):
#        pass