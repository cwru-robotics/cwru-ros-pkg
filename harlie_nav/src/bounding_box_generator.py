#!/usr/bin/env python

import roslib
roslib.load_manifest("harlie_nav")
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class BoundingBoxGenerator:
	def __init__(self):
		self.cloud = PointCloud()
		self.cloud.stamp = rospy.Time.now()
		self.cloud.frame_id = rospy.get_param("frame_id")
		self.origin = Point32()
		self.origin.x = rospy.get_param("origin_x")
		self.origin.y = rospy.get_param("origin_y")
		self.origin.z = rospy.get_param("origin_z")
		self.box_heading = rospy.get_param("box_heading")
		self.box_width = rospy.get_param("box_width")
		self.box_height = rospy.get_param("box_height")

if __name__ == "__main__":
	rospy.init_node("bounding_box_generator")
	bounding_box_gen = BoundingBoxGenerator()
	rospy.spin()
	 
