#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/vision_ws/devel/setup.bash

# 因为依赖底层摄像头，如果不包含则独立开节点
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_vision usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch robot_vision gesture_recognition.launch; exec bash"' \
