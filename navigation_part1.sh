#!/bin/bash

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/jason/livox_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_launch navigation.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; rosrun rm_global_planner path2waypoint; exec bash"
