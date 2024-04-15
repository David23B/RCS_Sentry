#!/bin/bash

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_launch local.launch; exec bash"
sleep 1

gnome-terminal -- bash -c "source /home/xmurcs/Desktop/rcs/devel/setup.bash; roslaunch rm_serial rm_serial.launch; exec bash"