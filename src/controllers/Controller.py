from abc import ABC, abstractmethod
import rospy

from geometry_msgs.msg import Point, Twist, Pose
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#from src.LidarScanner import LidarScanner
from .DiferentialRobot import DifferentialRobot

class ControlNode(ABC):

    def __init__(self,freq=10.0):
        # Init params
        rospy.init_node('control_node')
        #self.scanner = LidarScanner('/base_scan')
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Set time rate
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = freq
        self.rate = rospy.Rate(self.freq)

        # Set current goal
        self.goal = None
        self.U = (0,0,0)
        self.vel = Twist()
        rospy.Subscriber('/setup_goal',Point,self.callback_goal)

        #Set current position
        self.controller = DifferentialRobot(0.2)
        self.x = None
        self.y = None
        self.theta = 0
        rospy.Subscriber("/base_pose_ground_truth",Odometry, self.callback_pose_odometry) #
        rospy.Subscriber("/setup_origin",Pose,self.callback_pose)
        self.rate.sleep()

    def callback_goal(self,data):
        old_goal = self.goal
        self.goal = (data.x,data.y)
        if old_goal != self.goal:
            rospy.loginfo(f"Goal set to {self.goal}")
            self.rate.sleep()
            self.goal_update()

    # WaveFront and other methods that need extra computation
    def goal_update(self):
        return

    def callback_pose_odometry(self,data):
        #if self.x is not None and self.y is not None:
        self.callback_pose(data.pose.pose)

    def callback_pose (self,data):
        self.x = data.position.x
        self.y = data.position.y
        quat = data.orientation
        quat = [quat.x,quat.y,quat.z,quat.w]
        _,_,self.theta = euler_from_quaternion(quat)

    def publish_vel(self):
        self.pub_vel.publish(self.vel)

    def callback_time(self, data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6

    def run(self):
        self.setup()
        self.start()
        while not rospy.is_shutdown():
            self.iteration()
            self.rate.sleep()

    def setup(self):
        while self.goal is None or self.x is None or self.y is None:
            rospy.loginfo_once("Waiting for goal to be set")
            self.rate.sleep()

    def start(self):
        return
    
    @abstractmethod
    def iteration():
        return
