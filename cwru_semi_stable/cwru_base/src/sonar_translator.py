#!/usr/bin/env python
import roslib
roslib.load_manifest('cwru_base')
import rospy

from cwru_base.msg import Sonar
from sensor_msgs.msg import LaserScan

def handle_sonar(msg, scan_pub):
	scan = LaserScan()
	scan.header.frame_id = msg.header.frame_id
	scan.header.stamp = msg.header.stamp
	scan.angle_min = -0.19
	scan.angle_max = 0.19
	scan.angle_increment = 0.01266666667

	scan.time_increment = 0.0
	scan.scan_time = 0.05

	scan.range_min = 0.10
	scan.range_max = 2.0

	scan.ranges = [msg.dist for i in range(0,30)]
	scan_pub.publish(scan)

if __name__ == '__main__':
	rospy.init_node('harlie_sonar_translator')
	scan_pub = rospy.Publisher('sonar_scan', LaserScan)
	rospy.Subscriber('sonar', Sonar, handle_sonar, scan_pub)
	rospy.spin()
