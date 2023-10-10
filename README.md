# vpa_vision2action
This is the package that converts camera input to movement commands. The goal of this package is to be inclusive to all now or furture SNAM robots.

## launch files
### db19camera.launch
This launch files used the [usb_cam package](http://wiki.ros.org/usb_cam) to start the camera. 

### lane_follow.launch
This launch file subscribes to the "usb_cam/image_raw" topic.
The camera and chassis are enabled as well in this launch file, the lane following function starts.

#### lane following
The node find the yellow line and white line at lower part of the image and average the point's x coordinates. The lane center is determined and a PI controller is set to align the center (assumed to be the heading of the robot) to the lane center.

The PI controller is standalone in file lane_follow_controller.py with only P parameters in used at this moment. We set this structure of the purpose of teaching. The parameters are not in dynamic reconfiguration for now. (Student Tasks)

#### ACC
A blue (RGB:#2F5597) label is pasted at the rear of the robot. The vehicle will reduce speed when approaching from the rear. The distance is determined by the y-axis length of points. It also introduced some more criterion to make sure the car is in the same lane. The process is also controlled by a P(I) controller in the acc_controller.py

"cmd_vel" (Twist) is published at the frequency of the image_raw topic.

It is possible to turn off the acc function by the arg "acc_on:=false" when launching

#### stopline
Distance to stoplines are measured in this node but for now not in use. It will be used for possible application like intersection coordinations

## camera calibration
Please follow the instructions of [camera_calibration](http://wiki.ros.org/camera_calibration) to calibrate the camera. 

Please then store the calibration file as in config folder.
It is now by default using the fisheye_calibration.ini file in db19camera.launch


