#!/usr/bin/env python3
def lf_pi_control(ref,sig,err_intergal,t_gap):
    
    # kp: the p gain for this controller
    # ki: the i gain for this controller

    kp = 4
    ki = 0
    vf = 0.35
    vs = 0.35
    
    err = (ref - sig)/ref

    err_intergal += err * t_gap

    if sig/ref > 0.9 and sig/ref < 1.1:
        v_x = vf
        omega_z = 0
    else:
        v_x = vs
        omega_z = err * kp + err_intergal * ki

    return v_x,omega_z,err_intergal