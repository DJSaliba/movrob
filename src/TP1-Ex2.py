#!/usr/bin/env python3
import rospy
from controllers.Controller import ControlNode
from planners.PathFollower import PathFollower
import numpy as np

def cx(tk):
    t = tk % 182.717259
    if (t<=75):
        return t/10
    elif(t>75 and t<=152.511965):
        return 7.5 + 10.65294 * np.sin(2*np.pi*(t-75)/100)
    elif(t>152.511965):
        return -3.0205294 + (t-152.511965)/10
def cy(tk):
    t = tk % 182.717259
    if (t<=75):
        return 10*np.sin(2*np.pi*t/100)
    elif(t>75 and t<=152.511965):
        return -20.65294 + 10.65294 * np.cos(2*np.pi*(t-75)/100)
    elif(t>152.511965):
        return -18.9785434 + 2*np.pi*(t-152.511965)/10

class PathNode(ControlNode):
    def __init__(self,path):
        self.planner = PathFollower(path)
        super().__init__()

    def plan(self):
        self.U = self.planner.get_next([self.x,self.y])

if __name__ == "__main__":
    try:
        path = lambda t: np.array([-cx(t/250),cy(t/250)])/2
        node = PathNode(path)
        node.run()
    except rospy.ROSInterruptException:
        pass