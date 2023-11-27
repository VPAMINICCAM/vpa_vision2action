#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import numpy as np
# Image Processing
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Service
from vpa_vision2action.srv import AssignTask,InterManage,InterManageResponse
# Msgs
from geometry_msgs.msg import Twist
# Other files in this directory
from local_map import local_mapper
from lane_follow_controller import lf_pi_control
from acc_controller import acc_pi_control
# Dynamic reconfiguration
from vpa_vision2action.cfg import color_hsvConfig
from dynamic_reconfigure.server import Server
class HSVSpace:
    
    def __init__(self,h_u,h_l,s_u,s_l,v_u,v_l) -> None:
        self._h_upper = h_u
        self._h_lower = h_l
        self._s_upper = s_u
        self._s_lower = s_l
        self._v_upper = v_u
        self._v_lower = v_l
    
    def _generate_lower_mask(self):
        return np.array([self._h_lower,self._s_lower,self._v_lower])
    
    def _generate_upper_mask(self):
        return np.array([self._h_upper,self._s_upper,self._v_upper])
    
    def generate_mask(self,hsv_image):
        _mask   = cv2.inRange(hsv_image,self._generate_lower_mask(),self._generate_upper_mask())
        _kernel = np.ones((9,9),np.uint8)
        _mask   = cv2.morphologyEx(_mask,cv2.MORPH_CLOSE,_kernel)
        return _mask

class OpStatus:

    def __init__(self) -> None:

        self._pass_stopline         = False
        self._pause_flag            = False
        self._start_flag            = True
        self._is_yellow_left        = True
        self._is_in_intersection    = False
        self._request_inter_timer   = False
        self._task_index            = 0

        self._leaving_intersection  = False
        self._leaving_counter       = 0
        self._has_released          = False

        self._node_pointer      = 2 # point to the next node index

        self._task_list = []
    
    def loadNextAction(self):
        self._node_pointer += 1
        if self._node_pointer == len(self._task_list):
            # There is no more node to go
            self._next_action = -1 # no more next action
        else:
            self._setNode(node_type='last',node_value=self.this_node)
            self._setNode(node_type='this',node_value=self.next_node)
            self._setNode(node_type='next',node_value=self._task_list[self._node_pointer])
            self.setNextAction()

    def enterIntersection(self):
        self._is_in_intersection = True
    
    def loadTaskList(self,task_list:list):
        if len(task_list) < 3:
            rospy.loginfo('Invalid task list, no task loaded')
        else:
            self._setNode(node_type='last',node_value=task_list[0])
            self._setNode(node_type='this',node_value=task_list[1])
            self._setNode(node_type='next',node_value=task_list[2])
            self._task_list = task_list
    
    def setNextAction(self) -> None:
        self._next_action = local_mapper(self.last_node,self.this_node,self.next_node)

    def _setNode(self,node_type:str,node_value:int):
        
        if node_type == 'last':
            self.last_node = node_value
        elif node_type == 'this':
            self.this_node = node_value
        elif node_type == 'next':
            self.next_node = node_value
        else:
            rospy.loginfo("Invalid Node Type, No Value Set")

class AccControl:
    def __init__(self) -> None:
        self._acc_dis = 0
        self._acc_ref = 60
        self._last_valid_time = 0
        self._last_dis   = 0

class LaneOperationNode:

    def __init__(self,Nodename:str) -> None:
        
        rospy.init_node(Nodename)

        self._test_mode      = bool(rospy.get_param('~test_mode',False)) 
        # _test_mode will return the hsv value of the desired point, no vehicles action considered

        self._publish_mask   = bool(rospy.get_param('~publish_mask',True))
        # publish mask image for debug

        self._acc_mode   = bool(rospy.get_param('~acc_on',True))
        # simple ACC function on or off, default on

        self._robot_name = rospy.get_param('~robot_name','daisy')

        self.result_pub = rospy.Publisher("result_image", Image, queue_size=1)
        self.pub_cmd    = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        
        self._bridge = CvBridge()

        self._lane_hsv_1    = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_1',100)),
            h_l=int(rospy.get_param('~h_lower_1',80)),
            s_u=int(rospy.get_param('~s_upper_1',255)),
            s_l=int(rospy.get_param('~s_lower_1',80)),
            v_u=int(rospy.get_param('~v_upper_1',255)),
            v_l=int(rospy.get_param('~v_lower_1',150))
        ) # HSV space for yellow (center lane line)

        self._lane_hsv_2    = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_2',100)),
            h_l=int(rospy.get_param('~h_lower_2',25)),
            s_u=int(rospy.get_param('~s_upper_2',60)),
            s_l=int(rospy.get_param('~s_lower_2',0)),
            v_u=int(rospy.get_param('~v_upper_2',255)),
            v_l=int(rospy.get_param('~v_lower_2',200))
        ) # HSV space for white (side lane line)

        self._acc_hsv       = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_a',30)),
            h_l=int(rospy.get_param('~h_lower_a',0)),
            s_u=int(rospy.get_param('~s_upper_a',200)),
            s_l=int(rospy.get_param('~s_lower_a',90)),
            v_u=int(rospy.get_param('~v_upper_a',200)),
            v_l=int(rospy.get_param('~v_lower_a',20))
        )

        self._stop_line_hsv = HSVSpace(
            h_u=int(rospy.get_param('~h_upper_s',145)),
            h_l=int(rospy.get_param('~h_lower_s',110)),
            s_u=int(rospy.get_param('~s_upper_s',180)),
            s_l=int(rospy.get_param('~s_lower_s',120)),
            v_u=int(rospy.get_param('~v_upper_s',235)),
            v_l=int(rospy.get_param('~v_lower_s',170))
        )

        # guiding lines inside intersections - no dynamic reconfigure
        self._right_guide_hsv = HSVSpace(140,100,120,80,250,200)
        self._left_guide_hsv  = HSVSpace(160,140,180,90,230,160)
        self._thur_guide_hsv  = HSVSpace(30,0,250,190,220,170)   
        self._exit_line_hsv   = HSVSpace(50,20,240,200,220,150)

        self._veh = OpStatus()

        self._request_task_init()
        # pass task list to the OperationStatus item 'veh'
        self._veh.loadTaskList(task_list=self._request_task_service(task_index=0))
        self._veh.setNextAction()

        self._request_inter_init()
        
        if self._acc_mode:
            self._veh_acc = AccControl()
        
        if self._publish_mask:
            self.mask_pub_1 = rospy.Publisher("mask_1_image", Image, queue_size=1)
            self.mask_pub_2 = rospy.Publisher("mask_2_image", Image, queue_size=1)
            self.mask_pub_3 = rospy.Publisher("mask_stop_image", Image, queue_size=1)
            if self._acc_mode:
                self.mask_pub_acc = rospy.Publisher("mask_acc_image", Image, queue_size=1)
        
        self.image_sub  = rospy.Subscriber("usb_cam/image_raw", Image, self._image_cb)
        self._timer     = rospy.Timer(rospy.Duration(0.5),self._timer_cb)
        self.srv_color = Server(color_hsvConfig,self.dynamic_reconfigure_callback_hsv)
        
    def dynamic_reconfigure_callback_hsv(self,config,level):
        
        self._lane_hsv_1._h_lower = config.h_lower_1
        self._lane_hsv_1._s_lower = config.s_lower_1
        self._lane_hsv_1._v_lower = config.v_lower_1
        
        self._lane_hsv_1._h_upper = config.h_upper_1
        self._lane_hsv_1._s_upper = config.s_upper_1
        self._lane_hsv_1._v_upper = config.v_upper_1

        self._lane_hsv_2._h_lower = config.h_lower_2
        self._lane_hsv_2._s_lower = config.s_lower_2
        self._lane_hsv_2._v_lower = config.v_lower_2

        self._lane_hsv_2._h_upper = config.h_upper_2
        self._lane_hsv_2._s_upper = config.s_upper_2
        self._lane_hsv_2._v_upper = config.v_upper_2
        
        self._acc_hsv._h_lower = config.h_lower_a
        self._acc_hsv._s_lower = config.s_lower_a
        self._acc_hsv._v_lower = config.v_lower_a

        self._acc_hsv._h_upper = config.h_upper_a
        self._acc_hsv._s_upper = config.s_upper_a
        self._acc_hsv._v_upper = config.v_upper_a
        
        self._stop_line_hsv._h_lower = config.h_lower_s
        self._stop_line_hsv._s_lower = config.s_lower_s
        self._stop_line_hsv._v_lower = config.v_lower_s
        self._stop_line_hsv._h_upper = config.h_upper_s
        self._stop_line_hsv._s_upper = config.s_upper_s
        self._stop_line_hsv._v_upper = config.v_upper_s  
        
        return config

    def _timer_cb(self,_):
        if self._veh._request_inter_timer:
            _pass = self._request_inter_service(self._robot_name,self._veh._next_action,self._veh.this_node,self._veh.last_node,False)
            if _pass:
                print('Approved with timer')
                self._veh._pause_flag           = False
                self._veh._request_inter_timer  = False
                self._veh._has_released         = False

    def _image_cb(self,data:Image):
        try:
            # data.data   = data.data[0:int(0.75*len(data.data))]
            # data.height = int(0.75*data.height)
            cv_image    = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e) 
        cv_image = cv_image[int(cv_image.shape[0]/4):cv_image.shape[0],:] #removing the upper 25% of the image when processing
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)
        
        width_half  = int(hsv_image.shape[1]/2)
        height_half = int(hsv_image.shape[0]/2)
        
        if self._test_mode:
            # This mode is made for debug, reading back the hsv value and output on screen
            _test_image = self._draw_test_mark(width_half - 20,height_half + 30 ,hsv_image,cv_image)
            self._publish_image(self.result_pub,_test_image,False)
        else:
            if not self._veh._is_in_intersection:
                # the vehicle is not in the intersection
                # search for yellow and white line on both side
                # search for stop line

                mask1 = self._lane_hsv_1.generate_mask(hsv_image)
                mask2 = self._lane_hsv_2.generate_mask(hsv_image)

                mask_stop = self._stop_line_hsv.generate_mask(hsv_image)
                _line_center_1 = self._search_line(mask1,50,-20,height_half,10,0,width_half*2)
                if self._acc_mode:
                    mask_acc = self._acc_hsv.generate_mask(hsv_image)
                
                if self._veh._start_flag:
                    # This is the first frame
                    if _line_center_1 < width_half:
                        self._veh._is_yellow_left = True
                        rospy.loginfo('Initialized: Yellow Line on the left')
                    else:
                        self._veh._is_yellow_left = False
                        rospy.loginfo('Initialized: Yellow Line on the right')
                    self._veh._start_flag = False
                if self._veh._is_yellow_left and _line_center_1 == 0:
                    # The yellow is supposed to be on the left but we did not find it
                    _line_center_1 = 0
                elif not self._veh._is_yellow_left and _line_center_1 == 0:
                    # The yellow is supposed to be on the right but we did not find it
                    _line_center_1 = width_half * 2
                
                if self._veh._is_yellow_left: # Search the white line, ignoring what is on the other side of the yellow line
                    _line_center_2 = self._search_line(mask2,50,-20,height_half,10,_line_center_1,width_half*2)
                else:
                    _line_center_2 = self._search_line(mask2,50,-20,height_half,10,0,_line_center_1)
                if _line_center_2 == 0:
                    # we find no white line
                    if self._veh._is_yellow_left:
                        _line_center_2 = width_half * 2
                    else:
                        _line_center_2 = 0
                    
                _lane_center = int((_line_center_1 + _line_center_2)/2)
                cv2.circle(cv_image, (_lane_center,height_half), 5, (0,255,0), 5)

                # check distance to stopline
                _dis2stopline = self._distance_2_line(mask_stop,2*height_half,height_half,width_half)
                # print('stop line',_dis2stopline)
                if _dis2stopline > 30: # Tune me for distance
                    self._veh.enterIntersection()
                    # self._veh._pause_flag = True
                    if self._veh._next_action == -1:
                        # no action assigned for this intersection
                        self._veh._pause_flag = True
                        self._veh._task_index += 1
                        # ready for asking for more 
                        # disable cmd_veh 
                    elif not self._veh._request_inter_timer:
                        # There is action to perform but must ask for permission
                        _pass = self._request_inter_service(self._robot_name,self._veh._next_action,self._veh.this_node,self._veh.last_node,False)
                        if not _pass:
                            print("Did not get approved, swicth to timer")
                            self._veh._pause_flag = True
                            self._veh._request_inter_timer = True
                            # not approved for this request
                            # stop the car and switch to timer, ask again later
                        else:
                            self._veh._has_released = False
                            if self._veh._next_action == 0:
                                _text = 'go straight'
                            elif self._veh._next_action == 1:
                                _text = 'left turn'
                            elif self._veh._next_action == 2:
                                _text = 'right turn'
                            else:
                                _text = 'stop'
                            rospy.loginfo("Entering Intersection %s, action is %s",str(self._veh.this_node),_text)

            else:
                # we are in the intersection
                # look for another target
                if self._veh._next_action == -1:
                    # there is no other action
                    pass
                elif self._veh._next_action == 0:
                    _mask = self._thur_guide_hsv.generate_mask(hsv_image)
                elif self._veh._next_action == 1:
                    _mask = self._left_guide_hsv.generate_mask(hsv_image)
                elif self._veh._next_action == 2:
                    _mask = self._right_guide_hsv.generate_mask(hsv_image)
                else:
                    rospy.loginfo('Invalid action %s, failed to determine the guide line',str(self._veh._next_action))

                if self._veh._next_action in [0,1,2]:
                    _lane_center = self._search_guide_line(_mask,height_half)
                else:
                    _lane_center = 0
                
                if _lane_center == 0:
                    # no line found
                    pass
                else:
                    cv2.circle(cv_image, (_lane_center,height_half), 5, (255,255,0), 5)
                
                # checking if left intersection
                mask_exit      = self._exit_line_hsv.generate_mask(hsv_image)
                _dis2exitline  = self._distance_2_line(mask_exit,height_half*2,height_half,width_half)
                if _dis2exitline > 25:
                    self._veh._is_in_intersection = False
                    if not self._veh._has_released:
                        _pass = self._request_inter_service(self._robot_name,self._veh._next_action,self._veh.this_node,self._veh.last_node,True)
                        self._veh._has_released = True
                        rospy.loginfo('Exiting Intersection %s',str(self._veh.this_node))
                        self._veh.loadNextAction()
                        if self._veh._next_action == 0:
                            _text = 'straight'
                        elif self._veh._next_action == 1:
                            _text = 'left'
                        elif self._veh._next_action == 2:
                            _text = 'right'
                        else:
                            _text = 'stop'
                        rospy.loginfo('Next Action: %s',_text)
                        print('__________')

            # in an intersection or not, the acc shall always work
            if self._acc_mode:
                mask_acc = self._acc_hsv.generate_mask(hsv_image)
                self._search_front_car(mask_acc,height_half,rospy.get_time())
            if not _lane_center == 0:
                self._send_twist_command(_lane_center,width_half)
        # publish images for debug
        try:
            if self._publish_mask and not self._test_mode:
                if self._acc_mode:
                    self._publish_image(self.mask_pub_acc,mask_acc,True)
                
                if self._veh._is_in_intersection:
                    self._publish_image(self.mask_pub_3,mask_exit,True)
                    self._publish_image(self.mask_pub_1,_mask,True)
                else:
                    self._publish_image(self.mask_pub_3,mask_stop,True)
                    self._publish_image(self.mask_pub_1,mask1,True)
                    self._publish_image(self.mask_pub_2,mask2,True)

            self._publish_image(self.result_pub,cv_image,False)
        except CvBridgeError as e:
            print(e)

    def _draw_test_mark(self,width_select:int,height_select:int,hsv_image,res:Image) -> Image:
        cv2.circle(res, (width_select ,height_select), 5, (0,0,255), 1)
        cv2.line(res,(width_select -10, height_select), (width_select  +10,height_select), (0,0,255), 1)
        cv2.line(res,(width_select , height_select-10), (width_select , height_select+10), (0,0,255), 1)
        rospy.loginfo("Point HSV Value is %s"%hsv_image[height_select,width_select ])
        return res  

    def _request_task_init(self):
        rospy.loginfo("Waiting for task server")
        rospy.wait_for_service("/AssignTask")
        self._task_proxy = rospy.ServiceProxy("/AssignTask",AssignTask)
        rospy.loginfo("Task server online")
    
    def _request_task_service(self,task_index) -> list:
        resp = self._task_proxy(self._robot_name,task_index)
        return resp.node_list
    
    def _request_inter_init(self):
        rospy.loginfo("Waiting for intersection server")
        rospy.wait_for_service("/ManageInter")
        self._inter_proxy = rospy.ServiceProxy("/ManageInter",InterManage) 
        rospy.loginfo('Intersection manager online')

    def _request_inter_service(self,_robot_name:str,_next_action:int,_cur_node:int,_last_node:int,_is_release:bool) -> bool:
        _resp = self._inter_proxy(_robot_name,_next_action,_cur_node,_last_node,_is_release)
        return _resp.pass_flag

    def _search_line(self,_mask,_upper_bias:int,_lower_bias:int,_height_center:int,_interval:int,_width_range_left:int,_width_range_right:int) -> int:
        for i in range(_lower_bias,_upper_bias,_interval):
            point = np.nonzero(_mask[_height_center+i,_width_range_left:_width_range_right])[0] + _width_range_left
            if len(point) > 8 and len(point) < 45:
                _line_center = int(np.mean(point))
                return _line_center
            else:
                continue
        return 0 # nothing found in the end

    def _distance_2_line(self,_mask,_upper_bound:int,_lower_bound:int,_width_center:int) -> int:
        point = np.nonzero(_mask[_lower_bound:_upper_bound,_width_center])[0]
        if len(point) == 0:
            # did not find the line
            return 0
        elif len(point) > 5:
            return int(np.mean(point))
        else:
            return 0
        
    def _search_guide_line(self,_mask,_height_center:int):
        if self._veh._next_action == 1:
            low_bound   = 15
            upper_bound = 75
        elif self._veh._next_action == 0:
            low_bound   = 15
            upper_bound = 75
        else:
            low_bound = -15
            upper_bound = 75            
        for i in range(low_bound,upper_bound,15):
            seg_index = 0
            _l1 = np.nonzero(_mask[_height_center + i,:])[0]
            if len(_l1) > 10 and len(_l1) < 50:
                seg_dict  = {}
                cur_seg   = []
                last_pt   = 0
                for pt_index,pt in enumerate(_l1):
                    if pt_index == 0:
                        cur_seg.append(pt)
                        last_pt = pt
                    else:
                        if pt - last_pt < 5:
                            cur_seg.append(pt)
                            last_pt = pt
                        elif len(cur_seg) > 10 and len(cur_seg) < 25:
                            seg_dict[seg_index] = cur_seg
                            cur_seg = [pt]
                            last_pt = pt
                            seg_index += 1
                        else:
                            cur_seg = [pt]
                            last_pt = pt
                if len(cur_seg) > 10 and  len(cur_seg) < 25:
                    seg_dict[seg_index] = cur_seg         
                if seg_index >= 0:
                    # more than one segement found
                    # skip
                    # max_index = self._find_widest_seg(seg_dict)
                    if not self._veh._next_action == 1:
                        max_index = self._find_centerest_seg(seg_dict)
                    else:
                        max_index = 0
                    if max_index == -1:
                        continue
                    try:
                        return int(np.mean(seg_dict[max_index]))
                    except:
                        continue
                else:
                    continue
            else:
                continue
        return 0
    
    def _send_twist_command(self,center:int,ref:int):
        _twist2publish = Twist()
        [v_x,w_z,_] = lf_pi_control(ref,center,0,0)
        if self._acc_mode:
            [v_factor,_] = acc_pi_control(self._veh_acc._acc_ref,self._veh_acc._acc_dis,0,0)
        elif self._veh._is_in_intersection and self._veh._next_action == 2:
            pass
        else:
            v_factor = 1
        if self._veh._pause_flag:
            v_factor = 0
        _twist2publish.linear.x = v_x * v_factor 
        _twist2publish.angular.z = w_z * v_factor
        self.pub_cmd.publish(_twist2publish)

    def _search_front_car(self,_mask_acc,height_center:int,tick):
        _bias = -40
        _gap  = -10
        t_tol = 1
        p1 = np.nonzero(_mask_acc[height_center + _bias,:])[0]
        p2 = np.nonzero(_mask_acc[height_center + _bias + _gap,:])[0]

        if len(p1) > 10 and len(p2) > 10:
            # found a valid area
            c1 = int(np.mean(p1))
            c2 = int(np.mean(p2))
            if abs(c1-c2)/c1 < 0.2 and c1 > 60 and c1 < 290:
                self._veh_acc._last_dis         = self._veh_acc._acc_dis
                self._veh_acc._acc_dis          = len(np.nonzero(_mask_acc[:,c1])[0])
                self._veh_acc._last_valid_time  = tick
            else:
                now_seconds = rospy.get_time()
                if now_seconds - self._veh_acc._last_valid_time > t_tol:
                    self._veh_acc._acc_dis = 0
        else:
            now_seconds = rospy.get_time()
            if now_seconds - self._veh_acc._last_valid_time > t_tol:
                self._veh_acc._acc_dis = 0           

    def _publish_image(self,pub_name,image,_is_mask):
        if _is_mask:
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding="passthrough")
        else:
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        pub_name.publish(img_msg)

    def _find_widest_seg(self,seg_dict: dict):
        seg_len     = len(seg_dict)
        max_index   = 0
        try:
            max_len = len(seg_dict[0])
        except:
            return -1
        for i in range(1,seg_len):
            if len(seg_dict[i]) > max_len:
                max_len = len(seg_dict[i])
                max_index = i
        
        return max_index
    
    def _find_centerest_seg(self,seg_dict: dict):
        center_index    = 0
        seg_len         = len(seg_dict)
        try:
            center_value = abs(int(np.mean(seg_dict[0])) - 160)
        except:
            return -1
        for i in range(1,seg_len):
            if abs(np.mean(seg_dict[i]) - 160) < center_value:
                center_index = i
                center_value = abs(np.mean(seg_dict[i]) - 160)
        
        return center_index
        

if __name__ == '__main__':
    try:
        N = LaneOperationNode('Lane_Operation')
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()