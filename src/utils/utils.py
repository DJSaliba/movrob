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

def vec_angle(v0,v1,v2):
    v1,norm1 = vec_norm(v1,v0)
    v2,norm2 = vec_norm(v2,v0)
    u1 = v1/norm1
    u2 = v2/norm2
    theta = np.arcsin(np.clip(np.dot(u1, u2), -1.0, 1.0))
    if ccw(v0,v1,v2) > 0:
        theta -= np.pi
    return theta

def relative_angle(p1,p2,theta):
    v2, norm = vec_norm(p2,p1)
    u1 = np.array([1,0])
    u2 = v2/norm
    phi = np.arccos(np.clip(np.dot(u2, u1), -1.0, 1.0))
    angle = phi - theta
    while angle >=np.pi:
        angle -= np.pi
    while angle >=np.pi:
        angle += np.pi
    return angle

def ccw(A,B,C):
    cc = (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return 1 if cc else -1

def ortogonal_vec(v):
    return np.array([-v[1],v[0]])