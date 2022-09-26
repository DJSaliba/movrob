import rospy
from geometry_msgs.msg import Twist, Pose
from math import sqrt, sin, cos
from tf.transformations import euler_from_quaternion


#TODO: Convert example into class for reuse
class DifferentialRobot():
    def __init__(self,pose_topic,d):
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber(pose_topic,Pose, self.callback_pose)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.d = d

    def callback_pose (self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        _,_,self.theta = euler_from_quaternion(quat)

    #Rotina feedback linearization
    def feedback_linearization (self,U):
        v_x =  cos(self.theta)*U[0]         + sin(self.theta)*U[1]
        w_z = -sin(self.theta)*U[0]/self.d  + cos(self.theta)*U[1]/self.d
        return v_x, w_z
