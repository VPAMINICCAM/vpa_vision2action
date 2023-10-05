#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from vpa_vision2action.msg import ColorInfo
from dynamic_reconfigure.server import Server
from vpa_vision2action.cfg import color_hsvConfig
# This node will do all coloring processing

class color_detector():

    def __init__(self) -> None:

        rospy.init_node('color_detector')
        self.bridge = CvBridge()
        self.test_mode      = bool(rospy.get_param('~test_mode',False))
        self.mask_publish   = bool(rospy.get_param('~publish_mask',False))
        self.acc_mode       = bool(rospy.get_param('~acc_on',True))

        # get param from launch file 
        self.h_lower_1 = int(rospy.get_param('~h_lower_1',80))
        self.s_lower_1 = int(rospy.get_param('~s_lower_1',80))
        self.v_lower_1 = int(rospy.get_param('~v_lower_1',150))

        self.h_upper_1 = int(rospy.get_param('~h_upper_1',100))
        self.s_upper_1 = int(rospy.get_param('~s_upper_1',255))
        self.v_upper_1 = int(rospy.get_param('~v_upper_1',255))

        # a second filter is applied to detect the other boundary
        self.h_lower_2 = int(rospy.get_param('~h_lower_2',30))
        self.s_lower_2 = int(rospy.get_param('~s_lower_2',0))
        self.v_lower_2 = int(rospy.get_param('~v_lower_2',200))

        self.h_upper_2 = int(rospy.get_param('~h_upper_2',100))
        self.s_upper_2 = int(rospy.get_param('~s_upper_2',30))
        self.v_upper_2 = int(rospy.get_param('~v_upper_2',255))   

        # used to detect the car in front of the camera
        self.h_lower_a = int(rospy.get_param('~h_lower_a',110))
        self.s_lower_a = int(rospy.get_param('~s_lower_a',110))
        self.v_lower_a = int(rospy.get_param('~v_lower_a',60))

        self.h_upper_a = int(rospy.get_param('~h_upper_a',150))
        self.s_upper_a = int(rospy.get_param('~s_upper_a',180))
        self.v_upper_a = int(rospy.get_param('~v_upper_a',200))

        # stopline
        self.h_lower_s = int(rospy.get_param('~h_lower_s',110))
        self.s_lower_s = int(rospy.get_param('~s_lower_s',110))
        self.v_lower_s = int(rospy.get_param('~v_lower_s',200))

        self.h_upper_s = int(rospy.get_param('~h_upper_s',150))
        self.s_upper_s = int(rospy.get_param('~s_upper_s',180))
        self.v_upper_s = int(rospy.get_param('~v_upper_s',255))        

        # lane center point X Axis coordinate
        self.lane_center_1 = 0
        self.lane_center_2 = 0

        # stopline
        self.dis2stop = 0
        # frontcar
        if self.acc_mode:
            self.dis2car  = 0
            self.last_dis2car_time = 0

        self.image_width  = int(rospy.get_param('image_width',320))
        self.image_height = int(rospy.get_param('image_height',240))

        self.result_pub = rospy.Publisher("result_image", Image, queue_size=1)
        self.image_sub  = rospy.Subscriber("usb_cam/image_raw", Image, self._image_cb)

        self.color_pub  = rospy.Publisher("color_info",ColorInfo,queue_size=1)

        if self.mask_publish:
            self.mask_pub_1 = rospy.Publisher("mask_1_image", Image, queue_size=1)
            self.mask_pub_2 = rospy.Publisher("mask_2_image", Image, queue_size=1)
            self.mask_pub_3 = rospy.Publisher("mask_stop_image", Image, queue_size=1)
            if self.acc_mode:
                self.mask_pub_acc = rospy.Publisher("mask_acc_image", Image, queue_size=1)
        
        self.srv = Server(color_hsvConfig,self.dynamic_reconfigure_callback)

        rospy.loginfo('The color detector node has been initialized')

    def _image_cb(self,msg:Image):
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        except CvBridgeError as e:
            print(e)

        msg_to_pub = ColorInfo()

        self.lane_center_1 = 0
        self.lane_center_2 = 0

        # The top 25% of the image is not in use
        cv_image = cv_image[int(cv_image.shape[0]/4):cv_image.shape[0],:]
        # convert to hsv image format
        res = cv_image
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)

        width_half  = int(hsv_image.shape[1]/2)
        height_half = int(hsv_image.shape[0]/2)

        mask1     = self._mask_hsv_image(hsv_image,self.h_upper_1,self.s_upper_1,self.v_upper_1,self.h_lower_1,self.s_lower_1,self.v_lower_1)
        mask2     = self._mask_hsv_image(hsv_image,self.h_upper_2,self.s_upper_2,self.v_upper_2,self.h_lower_2,self.s_lower_2,self.v_lower_2)

        if self.acc_mode:
            mask_a = self._mask_hsv_image(hsv_image,self.h_upper_a,self.s_upper_a,self.v_upper_a,self.h_lower_a,self.s_lower_a,self.v_lower_a)
        
        mask_s = self._mask_hsv_image(hsv_image,self.h_upper_s,self.s_upper_s,self.v_upper_s,self.h_lower_s,self.s_lower_s,self.v_lower_s)

        if self.test_mode:
            # print the hsv value of teh center point
            cv2.circle(res, (width_half,height_half), 5, (0,0,255), 1)
            cv2.line(res,(width_half-10, height_half), (width_half +10,height_half), (0,0,255), 1)
            cv2.line(res,(width_half, height_half-10), (width_half, height_half+10), (0,0,255), 1)
            rospy.loginfo("Point HSV Value is %s"%hsv_image[height_half,width_half])
        else:
            # search yellow line
            for i in range(-20,50,10):
                point = np.nonzero(mask1[height_half + i])
                if len(point[0]) > 10:
                    try:
                        self.lane_center_1 = int(np.mean(point))
                    except:
                        pass
                    cv2.circle(res, (self.lane_center_1,height_half+i), 5, (255,0,0), 5)
                    break
            
            msg_to_pub.lane_yellow_center = self.lane_center_1

            # search white line
            for i in range(-20,50,10):
                point = np.nonzero(mask2[height_half + i])[0]
                if len(point) < 10:
                    continue

                point_right = point[point>self.lane_center_1] 
                point_left  = point[point<self.lane_center_1] 
                # We assume the thicker white point segements are the meaningful information
                # which means we should follow this landmark

                if len(point_right) > len(point_left) and len(point_right) > 10:
                    self.lane_center_2 = int(np.mean(point_right))
                    cv2.circle(res, (self.lane_center_2,height_half+i), 5, (0,255,0), 5)
                    break
                elif len(point_right) < len(point_left) and len(point_left) > 10:
                    self.lane_center_2 = int(np.mean(point_left))
                    cv2.circle(res, (self.lane_center_2,height_half+i), 5, (0,255,0), 5)
                    break
                else:
                    continue
            msg_to_pub.lane_white_center = self.lane_center_2

            # stopline distance
            point = np.nonzero(mask_s[height_half:2*height_half,width_half])
            if len(point[0]) == 0:
                self.dis2stop = 0
            else:
                self.dis2stop = int(np.mean(point))
            msg_to_pub.dis2stop = self.dis2stop

            # car distance detection            
            if self.acc_mode:
                i = -40
                h_d = -10
                t_gap_tol = 1
                point1 = np.nonzero(mask_a[height_half + i])
                point2 = np.nonzero(mask_a[height_half + i + + h_d])            
                if len(point1[0]) > 10 and len(point2[0]) > 10:
                    # sufficient detection
                    c1 = int(np.mean(point1))
                    c2 = int(np.mean(point2))                    
                    if abs(c1-c2)/c1 < 0.2 and c1 > 0.25*(2*width_half) and c1 < 0.75*(2*width_half):
                        # car detected is in the same lane
                        self.dis2car        = len(np.nonzero(mask_a[:,c1])[0])
                        self.last_dis2car_time = msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs
                    else:
                        # find nothing
                        now_seconds = rospy.get_time()
                        if now_seconds - self.last_dis2car_time > t_gap_tol:
                            # no car is in front, reset distance
                            self.dis2car = 0
                msg_to_pub.dis2car = self.dis2car
            else:
                msg_to_pub.dis2car = 0
            
            self.color_pub.publish(msg_to_pub)

            img_msg = self.bridge.cv2_to_imgmsg(res, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.result_pub.publish(img_msg)
        
        if self.mask_publish:
            img_msg = self.bridge.cv2_to_imgmsg(mask1, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub_1.publish(img_msg)
            
            img_msg = self.bridge.cv2_to_imgmsg(mask2, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub_2.publish(img_msg)

            img_msg = self.bridge.cv2_to_imgmsg(mask_s, encoding="passthrough")
            img_msg.header.stamp = rospy.Time.now()
            self.mask_pub_3.publish(img_msg)
            
            if self.acc_mode:
                img_msg = self.bridge.cv2_to_imgmsg(mask_a, encoding="passthrough")
                img_msg.header.stamp = rospy.Time.now()
                self.mask_pub_acc.publish(img_msg)

    def _mask_hsv_image(self,_hsv_img,_hu,_su,_vu,_hd,_sd,_vd):

        _lower = np.array([_hd,_sd,_vd])
        _upper = np.array([_hu,_su,_vu])
        
        _mask   = cv2.inRange(_hsv_img,_lower,_upper)

        _kernel = np.ones((9,9),np.uint8)

        _mask   = cv2.morphologyEx(_mask,cv2.MORPH_CLOSE,_kernel)
        
        return _mask

    def dynamic_reconfigure_callback(self,config,level):
        
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

if __name__ == "__main__":
    try:
        T = color_detector()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()