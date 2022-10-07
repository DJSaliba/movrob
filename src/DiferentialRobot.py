import rospy
from geometry_msgs.msg import Twist
from math import sqrt, sin, cos
import numpy as np

class DifferentialRobot():
    def __init__(self,setup_topic,vel_topic,pose_topic,d):
        self.d = d

    #Rotina feedback linearization
    def feedback_linearization(self,U,theta):
        vel = Twist()
        vel.linear.x  =  cos(theta)*U[0]        + sin(theta)*U[1]
        vel.angular.z = -sin(theta)*U[0]/self.d + cos(theta)*U[1]/self.d
        return vel
        
