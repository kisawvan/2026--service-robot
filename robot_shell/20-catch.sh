source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini auto_slam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini face_rec.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch match_mini catch.launch; exec bash"' \
