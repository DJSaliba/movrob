import numpy as np

def atractive_field(q,q_goal,ksi,d_sat):
    vec = q-q_goal
    norm = np.linalg.norm(vec)
    return ksi*vec if norm<=d_sat else d_sat*ksi*vec/norm

def repulsive_field(q,q_obj,eta,d_act):
    vec = q-q_obj
    norm = np.linalg.norm(vec)
    if norm >= d_act:
        return np.zeros([2,1])     
    else:
        return eta*(1/norm - 1/d_act)*vec/(norm**3)

def potential_field(q,q_goal,ksi,d_sat,q_obj,eta,d_act):
    Fa = atractive_field(q,q_obj,eta,d_act)
    Fr = repulsive_field(q,q_obj,eta,d_act)
    return Fa+Fr