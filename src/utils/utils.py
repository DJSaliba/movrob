import numpy as np

def euc_dist(p1,p2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    vec = p1-p2
    return vec, np.linalg.norm(vec)

def get_measured_points(x,y,theta,d):
    x = x + d* np.cos(theta)
    y = y + d* np.sin(theta)
    return (x,y)

def idxmin(l):
    return np.argmin(l)