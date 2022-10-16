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

def relative_angle(p1,p2,theta):
    v2, norm = vec_norm(p2,p1)
    u1 = np.array([1,0])
    u2 = v2/norm
    phi = np.arccos(np.clip(np.dot(u2, u1), -1.0, 1.0)) #* ccw([0,0],u1,u2)
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

# from: https://gist.github.com/nim65s/5e9902cd67f094ce65b0
def segment_dist(A, B, P):
    """ segment line AB, point P, where each one is an array([x, y]) """
    A = np.array(A)
    B = np.array(B)
    P = np.array(P)
    if all(A == P) or all(B == P):
        return 0
    if np.arccos(np.dot((P - A) / np.linalg.norm(P - A), (B - A) / np.linalg.norm(B - A))) > np.pi / 2:
        return np.linalg.norm(P - A)
    if np.arccos(np.dot((P - B) / np.linalg.norm(P - B), (A - B) / np.linalg.norm(A - B))) > np.pi / 2:
        return np.linalg.norm(P - B)
    return np.linalg.norm(np.cross(A-B, A-P))/np.linalg.norm(B-A)