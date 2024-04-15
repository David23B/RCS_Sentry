#!/bin/bash

sleep 3

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/jason/livox_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_launch mapping.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_serial rm_serial.launch; exec bash"
sleep 1