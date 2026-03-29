source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini face_rec.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini navigation_state.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini voice_controller.launch; exec bash"' \
