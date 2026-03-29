source /opt/ros/melodic/setup.bash
source ~/vision_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \

--tab -e 'bash -c "sleep 5; roslaunch robot_slam navigation.launch; exec bash"' \

--tab -e 'bash -c "sleep 5; roslaunch match_mini auto_slam3.launch; exec bash"' \

--tab -e 'bash -c "sleep 5; roslaunch robot_slam view_nav.launch; exec bash"' \
# 声控 去掉
--tab -e 'bash -c "sleep 5; roslaunch match_mini voice_controller.launch; exec bash"' \
# 连接到相机
--tab -e 'bash -c "sleep 5; roslaunch robot_vision usb_cam.launch; exec bash"' \
# 二维码识别
--tab -e 'bash -c "sleep 5; roslaunch robot_vision apriltag_detector.launch; exec bash"' \



