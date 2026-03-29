source /opt/ros/melodic/setup.bash
source ~/robot_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_vision usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch robot_vision take_photo.launch; exec bash"' \

