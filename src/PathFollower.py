import rospy
import numpy as np

class PathFollower():
    def reference_with_feedforward(self,position,path,t,dt):
        p = np.array(position[0:2]) # (x,y,theta) -> (x,y)
        target = path(t)
        err_p = target - p
        ff = (path(t+dt)-path(t-dt))/(2*dt)
        U = err_p + ff
        return U