from abc import ABC, abstractmethod
import rospy

from geometry_msgs.msg import Point, Twist
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#from src.LidarScanner import LidarScanner
from src.DiferentialRobot import DifferentialRobot

class ControlNode(ABC):

    def __init__(self,params,freq:float=10.0):
        # Init params
        self.set_simulation_params(params)
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
        self.controller = DifferentialRobot
        self.x = None
        self.y = None
        self.theta = 0
        rospy.Subscriber("/base_pose_ground_truth",Odometry, self.callback_pose) # 
        rospy.Subscriber("/setup_start",Odometry,self.callback_pose) # 

        self.rate.sleep()

    def set_sim_params(params):
        for k, v in params.items():
            rospy.set_param(k,v)

    def get_sim_param(k):
        return rospy.get_param(k)
    
    def callback_goal (self,data):
        self.goal = (data.pose.position.x,data.pose.position.y)
        self.goal_update()
    
    # WaveFront and other methods that need extra computation
    def goal_update():
        return

    def callback_pose (self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        _,_,self.theta = euler_from_quaternion(quat)

    def publish_vel(self):
        self.pub_vel(self.vel)

    def callback_time(self, data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6

    def run(self):
        self.setup()
        self.start()
        while not rospy.is_shutdown():
            self.iteration()
            self.rate.sleep()

    def setup(self):
        while self.goal is None or self.controller.x is None:
            self.rate.sleep()

    @abstractmethod
    def start():
        return
    
    @abstractmethod
    def iteration():
        return
