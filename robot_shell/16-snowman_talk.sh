source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch zoo_robot robot_lidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch snowman mini4_talk.launch; exec bash"' \

