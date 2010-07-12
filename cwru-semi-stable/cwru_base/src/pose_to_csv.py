#!/usr/bin/env python
import roslib
roslib.load_manifest('harlie_base')
import rospy

import tf
from harlie_base.msg import Pose

def handle_harlie_pose(msg, file):
    data_string = "%.50f, %.50f, %.50f, %.50f, %.50f, %.50f\n" % (msg.x, msg.x_var, msg.y, msg.y_var, msg.theta, msg.theta_var)
    file.write(data_string)

if __name__ == "__main__":
	rospy.init_node('pose_to_csv')
	with open(rospy.get_param("~filename"), "w") as file:
	    rospy.Subscriber('pose', Pose, handle_harlie_pose, file)
	    rospy.spin()
