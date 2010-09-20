#!/usr/bin/env python

# Copyright (c) 2010, Eric Perko
# All rights reserved
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import roslib
roslib.load_manifest('cwru_base')
import rospy

import tf
from cwru_base.msg import Pose

def handle_harlie_pose(msg, file):
    data_string = "%.50f, %.50f, %.50f, %.50f, %.50f, %.50f\n" % (msg.x, msg.x_var, msg.y, msg.y_var, msg.theta, msg.theta_var)
    file.write(data_string)

if __name__ == "__main__":
	rospy.init_node('pose_to_csv')
	with open(rospy.get_param("~filename"), "w") as file:
	    rospy.Subscriber('pose', Pose, handle_harlie_pose, file)
	    rospy.spin()
