import rospy
from sensor_msgs.msg import LaserScan


class LidarScanner():
    def __init__(self,scan_topic):
        self.ang_min = None
        self.ang_max = None
        self.increment = None
        self.rmin = None
        self.rmax = None
        self.ranges = []
        self.intensities = []
        rospy.Subscriber(scan_topic, LaserScan, self.callback_scan)