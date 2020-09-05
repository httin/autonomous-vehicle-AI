#!/bin/bash

. devel/setup.bash

rosrun data_pkg serialPort_node 

roslaunch data_pkg stereo_camera.launch 

roslaunch data_pkg record.launch 
