#!/usr/bin/env python3
def lf_pi_control(ref,sig,err_intergal,t_gap):
    
    # kp: the p gain for this controller
    # ki: the i gain for this controller

    kp = 0.5
    ki = 0
    vf = 0.2
    vs = 0.1
    
    err = (ref - sig)/ref
    print('err',err)
    err_intergal += err * t_gap

    if abs(sig/ref) > 0.95 and abs(sig/ref) < 1.05:
        v_x = vf
        omega_z = 0
    else:
        omega_z = err * kp + err_intergal * ki
        if abs(omega_z) < 0.2:
            v_x = vf - omega_z * 0.5
        else:
            v_x = vs
    return v_x,omega_z,err_intergal
