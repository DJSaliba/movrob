import numpy as np

def vec_norm(p1,p2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    vec = p1-p2
    return vec, np.linalg.norm(vec)

def manh_dist(p1,p2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    vec = p1-p2
    return np.linalg.norm(vec,1)

def get_measured_points(x,y,theta,d):
    x = x + d* np.cos(theta)
    y = y + d* np.sin(theta)
    return (x,y)

def idxmin(l):
    return np.argmin(l)

def vec_angle(p1,p2,theta):
    v2, norm = vec_norm(p1,p2)
    u1 = np.array([1,0])
    u2 = v2/norm
    phi = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
    if phi >=np.pi/2:
        phi -= np.pi
    return phi - theta