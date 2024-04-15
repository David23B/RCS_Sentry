#!/bin/bash

sleep 3

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/jason/livox_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_launch navigation.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; rosrun rm_global_planner path2waypoint; exec bash"
sleep 3

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0; exec bash"
sleep 5

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_launch local.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_serial rm_serial.launch; exec bash"