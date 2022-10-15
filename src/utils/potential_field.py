import numpy as np
from .utils import vec_norm

def atractive_field(q,q_goal,ksi,d_sat):
    vec,norm = vec_norm(q,q_goal)
    return -ksi*vec if norm<=d_sat else -d_sat*ksi*vec/norm

def repulsive_field(q,q_obj,eta,d_act):
    vec,norm = vec_norm(q,q_obj)
    if norm >= d_act:
        return np.zeros([2,1])
    else:
        return eta*(1/norm - 1/d_act)*vec/(norm**3)

def potential_field(q,q_goal,ksi,d_sat,q_obj,eta,d_act):
    Fa = atractive_field(q,q_goal,ksi,d_sat).reshape((2,))
    Fr = repulsive_field(q,q_obj,eta,d_act).reshape((2,))
    return Fa+Fr