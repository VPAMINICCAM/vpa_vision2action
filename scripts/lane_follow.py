#!/usr/bin/env python3

import rospy
from vpa_vision2action.msg import ColorInfo
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist
from lane_follow_controller import lf_pi_control
from acc_controller import acc_pi_control
from vpa_vision2action.cfg import lane_follow_piConfig
class lane_follow_cmd():
    
    def __init__(self) -> None:
       
       self.acc_mode    = bool(rospy.get_param('~acc_on',True))
       
       self._center     = int(rospy.get_param('usb_cam/image_width',240)/2)
       
       self._kp         = 5
       self._ki         = 0    
       self._vf         = 0.35 # Free travel speed
       self._vs         = 0.35 # Steering speed

       self._kp_acc     = 0.05
       self._ki_acc     = 0

       self._acc_ref    = 60 # pixel

       self._err_int        = 0 # for lane following
       self._err_int_acc    = 0
       self._last_time_lf   = 0
       self._last_time_acc  = 0

       rospy.init_node('lane_control')
       self.srv = Server(lane_follow_piConfig,self.dynamic_reconfigure_callback)
       self.color_sub   = rospy.Subscriber('color_info',ColorInfo,self._color_cb)
       self.pub_cmd     = rospy.Publisher('cmd_vel_lf',Twist,queue_size=1)
       rospy.loginfo('Lane and ACC control node ready')

    def _color_cb(self,msg:ColorInfo):
        
        _msg_to_pub     = Twist()
        _lane_center    = (msg.lane_yellow_center + msg.lane_white_center)/2

        if self._last_time_lf == 0:
            t_gap = 0
            self._last_time_lf = rospy.get_time()
        else:
            t_gap = rospy.get_time() - self._last_time_lf

        [v_x,w_z,self._err_int] = lf_pi_control(self._kp,self._ki,self._center,_lane_center,self._vf,self._vs,self._err_int,t_gap)

        if self.acc_mode:
            if self._last_time_acc == 0:
                t_gap = 0
                self._last_time_acc = rospy.get_time()
            else:
                t_gap = rospy.get_time() - self._last_time_acc

            v_factor = acc_pi_control(self._kp_acc,self._ki_acc,self._acc_ref,msg.dis2car,self._err_int_acc,t_gap)
            v_x = v_x * v_factor
            w_z = w_z * v_factor

        _msg_to_pub.linear.x    = v_x
        _msg_to_pub.angular.z   = w_z
        self.pub_cmd.publish(_msg_to_pub)

    def dynamic_reconfigure_callback(self,config,level):
        self._kp = config.kp_lf
        self._ki = config.ki_lf
        self._vf = config.vf
        self._vs = config.vs

        self._kp_acc = config.kp_acc
        self._ki_acc = config.ki_acc
        self._acc_ref = config.acc_ref
        return config

if __name__ == "__main__":
    try:
        T = lane_follow_cmd()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")