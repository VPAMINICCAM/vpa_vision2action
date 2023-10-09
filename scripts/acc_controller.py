#!/usr/bin/env python3
def acc_pi_control(ref,sig,err_intergal,t_gap):

    kp = 0.05
    ki = 0
    
    err = sig -ref

    err_intergal += err * t_gap

    v_factor = 1 - (err * kp + err_intergal * ki)

    if v_factor > 1:
        v_factor = 1
    elif v_factor < 0:
        v_factor = 0
    
    return v_factor,err_intergal