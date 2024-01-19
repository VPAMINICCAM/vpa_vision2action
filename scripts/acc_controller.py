#!/usr/bin/env python3
class AccControl:
    def __init__(self) -> None:
        self._acc_dis = 0
        self._acc_ref = 50
        self._last_valid_time = 0
        self._last_dis   = 0

    def acc_pi_control(self,ref,sig,err_intergal,t_gap):

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