#!/bin/bash
source /opt/ros/melodic/setup.bash
# source both workspaces
source ~/robot_ws/devel/setup.bash
source ~/vision_ws/devel/setup.bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; roslaunch robot_vision usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch robot_vision gesture_recognition.launch; exec bash"' \