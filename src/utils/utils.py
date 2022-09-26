import numpy as np


def euc_dist(p1,p2):
    p1 = np.array(p1)
    p2 = np.array(p2)
    return np.linalg.norm(p1-p2)

