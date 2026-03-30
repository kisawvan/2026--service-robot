source /opt/ros/melodic/setup.bash
source ~/vision_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
"开启雷达"
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
"开启导航"
--tab -e 'bash -c "sleep 5; roslaunch robot_slam navigation.launch; exec bash"' \
"开启导航视图"    
--tab -e 'bash -c "sleep 5; roslaunch robot_slam view_nav.launch; exec bash"' \
"连接底盘"
--tab -e 'bash -c "sleep 5; roslaunch zoo_bringup bringup_with_imu.launch; exec bash"' \
"开启键盘控制"
--tab -e 'bash -c "sleep 5; roslaunch zoo_control keyboard_teleop.launch; exec bash"' \
