from abc import ABC, abstractmethod
import rospy
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock

class ControlNode(ABC):

    def __init__(self,params,freq:float=10.0):
        self.set_simulation_params(params)
        rospy.init_node('control_node')
        rospy.Subscriber("/goal",Pose,self.callback_goal)
        
        rospy.Subscriber('/clock', Clock, self.callback_time)
        self.freq = freq
        self.rate = rospy.Rate(self.freq)

        self.sim_params = self.get_sim_params()
        self.U = (0,0,0)
        self.cur_target = (0,0,0)
        self.rate.sleep()

    def callback_goal (self,data):
        pass

    def set_sim_params(params):
        for k, v in params.items():
            rospy.set_param(k,v)
    
    def get_sim_param(k):
        return rospy.get_param(k)

    def callback_time(self, data):
        self.time = data.clock.secs*1e3 + data.clock.nsecs/1e6

    @abstractmethod
    def iteration():
        return

    @abstractmethod
    def target():
        return

    def run(self):
        while not rospy.is_shutdown():
            self.iteration()
            self.rate.sleep()