#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import roslaunch

def ifrelocal_callback(msg):
    if msg.data:
        rospy.loginfo("Received True message on /ifrelocal topic. Starting relocalization.launch.")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(["vehicle_simulator", "relocalization.launch"])
        roslaunch_args = launch_file[2:]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_args)
        parent.start()
        rospy.loginfo("Relocalization launched.")

def ifrelocal_listener():
    rospy.init_node('ifrelocal_listener', anonymous=True)
    rospy.Subscriber("/ifrelocal", Bool, ifrelocal_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ifrelocal_listener()
    except rospy.ROSInterruptException:
        pass

