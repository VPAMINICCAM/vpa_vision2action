# vpa_vision2action
This is the package that converts camera input to movement commands. The goal of this package is to be inclusive to all now or furture SNAM robots.

## launch files
### db19camera.launch
This launch files used the [usb_cam package](http://wiki.ros.org/usb_cam) to start the camera. 

### color_land_mark_detector.launch
This launch file subscribes to the "usb_cam/image_raw" topic to get the following information
+ the x-axis center of yellow line (pixel)
+ the x-axis center of white line (pixel) 
+ the distance to the next stop line (pixel), the bigger the closer
+ the distance to the car in front, the bigger the closer

## camera calibration
Please follow the instructions of [camera_calibration](http://wiki.ros.org/camera_calibration) to calibrate the camera. 

Please then store the calibration file as in config folder.
It is now by default using the fisheye_calibration.ini file in db19camera.launch
