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

from cwru_base.msg import Sonar
from sensor_msgs.msg import LaserScan

scan=LaserScan()

def handle_sonar(msg, scan_pub):
	scan.header.frame_id = msg.header.frame_id
	scan.header.stamp = msg.header.stamp
	scan.ranges = [msg.dist for i in range(0,30)]
	scan_pub.publish(scan)

if __name__ == '__main__':
	rospy.init_node('sonar_translator')
	scan_pub = rospy.Publisher('sonar_scan', LaserScan)
	rospy.Subscriber('sonar', Sonar, handle_sonar, scan_pub)


	scan.angle_min = rospy.get_param("~angle_min", -0.19)
	scan.angle_max = rospy.get_param("~angle_max", 0.19)
	scan.angle_increment = rospy.get_param("~angle_increment", 0.01266666667)

	scan.time_increment = rospy.get_param("~time_increment", 0.0)
	scan.scan_time = rospy.get_param("~scan_time", 0.05)

	scan.range_min = rospy.get_param("~range_min", 0.10)
	scan.range_max = rospy.get_param("~range_max", 2.0)

	rospy.spin()
