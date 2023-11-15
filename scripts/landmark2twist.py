#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from vpa_vision2action.cfg import color_hsvConfig
from dynamic_reconfigure.server import Server
from lane_follow_controller import lf_pi_control
from acc_controller import acc_pi_control
from vpa_vision2action.srv import AssignTask
from vpa_vision2action.srv import InterManage
from local_map import local_mapper
class lane_follow:
    def __init__(self):    
        #define topic publisher and subscriber
        self.bridge         = CvBridge()  
        self.test_mode      = bool(rospy.get_param('~test_mode',False))
        self.publish_mask   = bool(rospy.get_param('~publish_mask',True))
        self.acc_mode   = bool(rospy.get_param('~acc_on',True))
        self.pub_cmd    = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.robot_name = rospy.get_param('~robot_name','vivian')
        self.start_flag = True
        self.yellowleft = True
        
        self.result_pub = rospy.Publisher("result_image", Image, queue_size=1)
        
        if self.publish_mask:
            self.mask_pub_1 = rospy.Publisher("mask_1_image", Image, queue_size=1)
            self.mask_pub_2 = rospy.Publisher("mask_2_image", Image, queue_size=1)
            self.mask_pub_3 = rospy.Publisher("mask_stop_image", Image, queue_size=1)
            if self.acc_mode:
                self.mask_pub_acc = rospy.Publisher("mask_acc_image", Image, queue_size=1)
        
        # get param from launch file 
        self.h_lower_1 = int(rospy.get_param('~h_lower_1',80))
        self.s_lower_1 = int(rospy.get_param('~s_lower_1',80))
        self.v_lower_1 = int(rospy.get_param('~v_lower_1',150))

        self.h_upper_1 = int(rospy.get_param('~h_upper_1',100))
        self.s_upper_1 = int(rospy.get_param('~s_upper_1',255))
        self.v_upper_1 = int(rospy.get_param('~v_upper_1',255))

        # a second filter is applied to detect the other boundary
        self.h_lower_2 = int(rospy.get_param('~h_lower_2',20))
        self.s_lower_2 = int(rospy.get_param('~s_lower_2',0))
        self.v_lower_2 = int(rospy.get_param('~v_lower_2',200))

        self.h_upper_2 = int(rospy.get_param('~h_upper_2',100))
        self.s_upper_2 = int(rospy.get_param('~s_upper_2',30))
        self.v_upper_2 = int(rospy.get_param('~v_upper_2',255))
        
        self._err_int_lf = 0
        
        self.lane_center_1  = 0
        self.lane_center_2  = 0   
        self.center_point   = 0
        # acc
        self.h_lower_a = int(rospy.get_param('~h_lower_a',5))
        self.s_lower_a = int(rospy.get_param('~s_lower_a',130))
        self.v_lower_a = int(rospy.get_param('~v_lower_a',100))

        self.h_upper_a = int(rospy.get_param('~h_upper_a',50))
        self.s_upper_a = int(rospy.get_param('~s_upper_a',200))
        self.v_upper_a = int(rospy.get_param('~v_upper_a',200))
        
       # stop_line
        self.h_lower_s = int(rospy.get_param('~h_lower_s',110))
        self.s_lower_s = int(rospy.get_param('~s_lower_s',110))
        self.v_lower_s = int(rospy.get_param('~v_lower_s',170))

        self.h_upper_s = int(rospy.get_param('~h_upper_s',150))
        self.s_upper_s = int(rospy.get_param('~s_upper_s',180))
        self.v_upper_s = int(rospy.get_param('~v_upper_s',235))          

        # guide line
        self.h_lower_g = [5,    130,    10] # through, left , right
        self.s_lower_g = [80,   40,     10]
        self.v_lower_g = [160,  105,    160]
        
        self.h_upper_g = [30,   170,    40]
        self.s_upper_g = [105,   75,    30]
        self.v_upper_g = [210,  200,    240]
        
        # intersection exit
        self.h_lower_e = 35
        self.s_lower_e = 50
        self.v_lower_e = 140
        
        self.h_upper_e = 90
        self.s_upper_e = 130
        self.v_upper_e = 255
        self._pause_flag = False
        
        rospy.loginfo("Waiting for task server")
        rospy.wait_for_service("/AssignTask")
        
        self._task_proxy = rospy.ServiceProxy("/AssignTask",AssignTask)
        resp = self._task_proxy(self.robot_name,0)
        self._node_list = resp.node_list
        if len(self._node_list) == 0:
            self._next_action = -1
            rospy.loginfo('No task assigned for %s ',self.robot_name)
        else:
            self._last_node     = self._node_list[0]
            self._current_node  = self._node_list[1]
            self._next_node     = self._node_list[2]
            self._next_action = local_mapper(self._last_node,self._current_node,self._next_node)
            rospy.loginfo('Node list:%s ',str(self._node_list))
        self._node_index   = 2
        self._task_index   = 0
        
        self.stopline_dis  = 0
        self._intersection_flag = False

        rospy.loginfo("Waiting for intersection server")
        rospy.wait_for_service("/ManageInter")
        self._inter_proxy = rospy.ServiceProxy("/ManageInter",InterManage)  
        #debug
        # self._intersection_flag = True
        # self._next_action = 2
        
        if self.acc_mode:
            if self.test_mode:
                self.mask_pub_acc = rospy.Publisher("mask_acc_image", Image, queue_size=1)
                
            self.acc_dis = 0
            self.acc_ref = 60
            self.err_int_acc = 0
            self.last_distance = 0
        
            self.acc_kp  = 0.05
            self.acc_ki  = 0
            self.last_vaild_time = 0
        
        self._timer    = rospy.Timer(rospy.Duration(0.5),self._timer_cb)    
        self.srv_color = Server(color_hsvConfig,self.dynamic_reconfigure_callback_hsv)
        self.image_sub  = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        
        
    def dynamic_reconfigure_callback_hsv(self,config,level):
        # update config param
        self.h_lower_1 = config.h_lower_1
        self.s_lower_1 = config.s_lower_1
        self.v_lower_1 = config.v_lower_1

        self.h_upper_1 = config.h_upper_1
        self.s_upper_1 = config.s_upper_1
        self.v_upper_1 = config.v_upper_1

        self.h_lower_2 = config.h_lower_2
        self.s_lower_2 = config.s_lower_2
        self.v_lower_2 = config.v_lower_2

        self.h_upper_2 = config.h_upper_2
        self.s_upper_2 = config.s_upper_2
        self.v_upper_2 = config.v_upper_2

        self.h_lower_a = config.h_lower_a
        self.s_lower_a = config.s_lower_a
        self.v_lower_a = config.v_lower_a

        self.h_upper_a = config.h_upper_a
        self.s_upper_a = config.s_upper_a
        self.v_upper_a = config.v_upper_a

        self.h_lower_s = config.h_lower_s
        self.s_lower_s = config.s_lower_s
        self.v_lower_s = config.v_lower_s
        self.h_upper_s = config.h_upper_s
        self.s_upper_s = config.s_upper_s
        self.v_upper_s = config.v_upper_s  
        
        return config
    def _timer_cb(self,_):
        if self._intersection_flag and self._pause_flag:
            if not self._next_action == -1:
                resp = self._inter_proxy(self.robot_name,self._next_action,self._current_node,self._last_node,False)
                if resp.pass_flag:
                    self._pause_flag = False
    
    def callback(self,data:Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv_image = cv_image[int(cv_image.shape[0]/4):cv_image.shape[0],:]
        res = cv_image
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)
        if not self._intersection_flag:
        #set color mask min amd max value
            lane_lower_1 = np.array([self.h_lower_1,self.s_lower_1,self.v_lower_1])
            lane_upper_1 = np.array([self.h_upper_1,self.s_upper_1,self.v_upper_1])
            lane_lower_2 = np.array([self.h_lower_2,self.s_lower_2,self.v_lower_2])
            lane_upper_2 = np.array([self.h_upper_2,self.s_upper_2,self.v_upper_2])  
            # get mask from color
            mask1 = cv2.inRange(hsv_image,lane_lower_1,lane_upper_1)
            mask2 = cv2.inRange(hsv_image,lane_lower_2,lane_upper_2)
            # close operation to fit some little hole
            kernel = np.ones((9,9),np.uint8)
            mask1 = cv2.morphologyEx(mask1,cv2.MORPH_CLOSE,kernel)
            mask2 = cv2.morphologyEx(mask2,cv2.MORPH_CLOSE,kernel)
            
            stop_lower = np.array([self.h_lower_s,self.s_lower_s,self.v_lower_s])
            stop_upper = np.array([self.h_upper_s,self.s_upper_s,self.v_upper_s])
            mask_stop  = cv2.inRange(hsv_image,stop_lower,stop_upper)
            
        # if test mode,output the center point HSV value
        if self.acc_mode:
            kernel = np.ones((9,9),np.uint8)
            line_lower = np.array([self.h_lower_a,self.s_lower_a,self.v_lower_a])
            line_upper = np.array([self.h_upper_a,self.s_upper_a,self.v_upper_a])
            mask = cv2.inRange(hsv_image,line_lower,line_upper)
            mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        width_half  = int(hsv_image.shape[1]/2)
        height_half = int(hsv_image.shape[0]/2)
        
        if self.test_mode:
            # it print the hsv value of the center point in this image
            # play around here to move the cursor
            width_select    = width_half  - 10
            height_select   = height_half + 30
            cv2.circle(res, (width_select ,height_select), 5, (0,0,255), 1)
            cv2.line(res,(width_select -10, height_select), (width_select  +10,height_select), (0,0,255), 1)
            cv2.line(res,(width_select , height_select-10), (width_select , height_select+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%hsv_image[height_select,width_select ])            
        else:
            if not self._intersection_flag:
                self.lane_center_1 = self._search_yellow_line(mask1,height_half)
                cv2.circle(res, (self.lane_center_1,height_half), 5, (255,0,0), 5)
            
                # By using the yellow line (center line of lane) information,
                # Some background information will be removed and the car knows which lane it is on
                if self.start_flag:
                    if self.lane_center_1 > width_half:
                        self.yellowleft = False
                        rospy.loginfo('Initialized: Yellow Line on the right')
                    else:
                        rospy.loginfo('Initialized: Yellow Line on the left')
                    self.start_flag = False
            
                self.lane_center_2  = self._search_white_line(mask2,height_half,self.lane_center_1,self.yellowleft)
                cv2.circle(res, (self.lane_center_1,height_half), 5, (0,255,0), 5)
                self.center_point   = self._determine_lanecenter(self.lane_center_1,self.lane_center_2,width_half*2)
                    
            # getting distance to stop line    
                point = np.nonzero(mask_stop[height_half:2*height_half,width_half])
                if len(point[0]) == 0:
                    self.stopline_dis = 0
                else:
                    self.stopline_dis = int(np.mean(point))
                # print(self.stopline_dis)
                if self.stopline_dis > 45:
                    # Approaching the intersection
                    # Switch to line following
                    self._intersection_flag = True
                    if not self._next_action == -1:
                        resp = self._inter_proxy(self.robot_name,self._next_action,self._current_node,self._last_node,False)
                        if not resp.pass_flag:
                            self._pause_flag = True
                    if self._next_action == -1:
                        self._pause_flag = True
                    else:
                        print('Intersection: ',self._current_node)
                        print('Intersection action:',self._next_action)
            else:
                # now chose another target
                if not self._next_action == -1:
                    lane_lower = np.array([self.h_lower_g[self._next_action],self.s_lower_g[self._next_action],self.v_lower_g[self._next_action]])
                    lane_upper = np.array([self.h_upper_g[self._next_action],self.s_upper_g[self._next_action],self.v_upper_g[self._next_action]])
                    
                    mask_guide  = cv2.inRange(hsv_image,lane_lower,lane_upper)
                    kernel      = np.ones((9,9),np.uint8)
                    mask_guide  = cv2.morphologyEx(mask_guide,cv2.MORPH_CLOSE,kernel)               
                    
                    self.center_point = self._search_guide_line(mask_guide,height_half)
                    
                    # checking the exit of intersection
                    exit_lower = np.array([self.h_lower_e,self.s_lower_e,self.v_lower_e])
                    exit_upper = np.array([self.h_upper_e,self.s_upper_e,self.v_upper_e])
                    
                    mask_exit   = cv2.inRange(hsv_image,exit_lower,exit_upper)
                    point = np.nonzero(mask_exit[height_half:2*height_half,width_half])
                    if len(point[0]) == 0:
                        exit_dis = 0
                    else:
                        exit_dis = int(np.mean(point))
                        
                    if exit_dis > 15 and len(point[0]) > 5:
                        self._intersection_flag = False
                        print('Exiting intersection')
                        resp = self._inter_proxy(self.robot_name,self._next_action,self._current_node,self._last_node,True)
                        self._last_node     = self._current_node
                        self._current_node  = self._next_node
                        self._node_index += 1
                        if self._node_index == len(self._node_list):
                            # no more action needed
                            self._next_action = -1
                            self._task_index  += 1
                            print('Task Finished')
                        else:
                            self._next_node     = self._node_list[self._node_index]
                            self._next_action   = local_mapper(self._last_node,self._current_node,self._next_node)
                            print('Next node: ',self._current_node)
                            print('____')
                else:
                    self._pause_flag = True
                
            # ACC function starts here
            if self.acc_mode:
                i = -40
                h_d = -10
                t_gap_tol = 1
                point = np.nonzero(mask[height_half + i])  
                point2 = np.nonzero(mask[height_half + i + + h_d])             
                if len(point[0]) > 10 and len(point2[0]) > 10:
                    c1 = int(np.mean(point))
                    c2 = int(np.mean(point2))
                    if abs(c1-c2)/c1 < 0.2 and c1 > 60 and c1 < 290:
                        cv2.line(res,(50,height_half+ i),(270,height_half + i),(180,60,200),1)
                        self.last_distance = self.acc_dis
                        self.acc_dis = len(np.nonzero(mask[:,c1])[0])
                        self.last_vaild_time = data.header.stamp.secs + 1e-9 * data.header.stamp.nsecs
                    else:
                        now_seconds = rospy.get_time()
                        if now_seconds - self.last_vaild_time > t_gap_tol:
                            self.acc_dis = 0
                else:
                    now_seconds = rospy.get_time()
                    if now_seconds - self.last_vaild_time > t_gap_tol:
                        self.acc_dis = 0
                
            if self.center_point:
                cv2.circle(res,(int(self.center_point),height_half),5,(0,0,255),5)
                _twist2publish = Twist()
                [v_x,w_z,self._err_int_lf] = lf_pi_control(width_half,self.center_point,self._err_int_lf,0)
                
                if self.acc_mode:
                    [v_factor,self.err_int_acc] = acc_pi_control(self.acc_ref,self.acc_dis,self.err_int_acc,0)
                else:
                    v_factor = 1
                _twist2publish.linear.x = v_x * v_factor 
                _twist2publish.angular.z = w_z * v_factor
                if not self._pause_flag:
                    self.pub_cmd.publish(_twist2publish)
                else:
                    _twist2publish = Twist()
                    self.pub_cmd.publish(_twist2publish)
                
            self.center_point = width_half 
            
            if self.yellowleft:
                self.lane_center_1 = 0
                self.lane_center_2 = width_half * 2
            else:
                self.lane_center_1 = width_half * 2
                self.lane_center_2 = 0
                
        try:
            if self.publish_mask:
                if not self._intersection_flag:
                    img_msg = self.bridge.cv2_to_imgmsg(mask1, encoding="passthrough")
                    img_msg.header.stamp = rospy.Time.now()
                    self.mask_pub_1.publish(img_msg)

                    img_msg = self.bridge.cv2_to_imgmsg(mask2, encoding="passthrough")
                    img_msg.header.stamp = rospy.Time.now()
                    self.mask_pub_2.publish(img_msg)

                    img_msg = self.bridge.cv2_to_imgmsg(mask_stop, encoding="passthrough")
                    img_msg.header.stamp = rospy.Time.now()
                    self.mask_pub_3.publish(img_msg)
                elif not self.test_mode:
                    img_msg = self.bridge.cv2_to_imgmsg(mask_guide, encoding="passthrough")
                    img_msg.header.stamp = rospy.Time.now()
                    self.mask_pub_1.publish(img_msg)   # reuse the first mask topic for debugging guide line mask
                    
                    img_msg = self.bridge.cv2_to_imgmsg(mask_exit, encoding="passthrough")
                    img_msg.header.stamp = rospy.Time.now()
                    self.mask_pub_3.publish(img_msg)
                    
                img_msg = self.bridge.cv2_to_imgmsg(mask, encoding="passthrough")
                img_msg.header.stamp = rospy.Time.now()
                self.mask_pub_acc.publish(img_msg)  

            img_msg = self.bridge.cv2_to_imgmsg(res, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.result_pub.publish(img_msg)
            
        except CvBridgeError as e:
            print(e)
        
    def _search_yellow_line(self,mask,half_h):
        for i in range(-20,50,10):
            point = np.nonzero(mask[half_h + i])
            if len(point[0]) > 10:
                _lane_center_1 = int(np.mean(point))
                return _lane_center_1
        return 0
    
    def _search_white_line(self,mask,half_h,yellow_line,yellow_left):
        for i in range(-20,50,10):
            point = np.nonzero(mask[half_h + i])[0]
            if len(point) > 10:
                if yellow_left:
                    point = point[point>yellow_line]
                    if len(point) == 0:
                        continue
                else:
                    point = point[point<yellow_line]
                    if len(point) == 0:
                        continue
                return int(np.mean(point))
        return 0 
    
    def _determine_lanecenter(self,l1,l2,w):
        if l1 * l2 > 0:
            return (l1+l2)/2
            # both lines found
        elif l1 > 0:
            # only found yellow line
            if self.yellowleft:
                return (l1 + w)/2
            else:
                return l1/2
        elif l1+l2 == 0:
            # miss both detections
            return w
        else:
            # only found white line
            if self.yellowleft:
                return l2/2
            else:
                return (l2+w)/2
            
    def _search_guide_line(self,mask,half_h):
        if self._next_action == 1:
            range_search = range(20,80,10)
        elif self._next_action == 2:
            range_search = range(0,50,10)
        else:
            range_search = range(0,50,10)
        for i in range_search:
            point = np.nonzero(mask[half_h + i])
            if self._next_action == 0:
                point1 = np.nonzero(mask[half_h + i + 20])
            #print(len(point[0]))
            if not self._next_action == 0:
                if len(point[0]) > 8 and len(point[0]) < 50: # Remove lines of other directions
                    data = point[0]
                    seg_index = 0
                    seg_dict = {}
                    cur_seg = []
                    for pt in range(len(data)):
                        if pt == 0:
                            cur_seg.append(data[pt])
                        else:
                            if data[pt] - data[pt-1] < 5:
                                # continous
                                cur_seg.append(data[pt])
                            elif len(cur_seg) > 8:
                                seg_dict[seg_index] = cur_seg
                                cur_seg = [data[pt]]
                                seg_index += 1
                            else:
                                cur_seg = [data[pt]]
                                # too short, abandon
                    if len(cur_seg) > 8:
                        seg_dict[seg_index] = cur_seg
                    if len(seg_dict) == 0:
                        return 0
                    if self._next_action == 1:
                        _line_center = int(np.mean(seg_dict[0]))
                    elif self._next_action == 2 and len(seg_dict) > 1:
                        _line_center = int(np.mean(seg_dict[1]))
                    else:
                        _line_center = int(np.mean(seg_dict[0]))
                    return _line_center
            else:
                # straight line
                if len(point1[0]) > 8 and len(point1[0]) < 50:
                    _line_center = np.mean(point1)
                    return _line_center
                else:
                    continue
        return 0
    
if __name__ == '__main__':
    try:
        # init ROS node 
        rospy.init_node("lane_follow")
        rospy.loginfo("Starting Lane Follow node")
        lane_follow()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()