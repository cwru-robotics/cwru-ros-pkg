#!/usr/bin/env python

import roslib
roslib.load_manifest("harlie_nav")
import rospy

import math

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
import tf

class BoundingBoxGenerator:
	def __init__(self):
		self.box_width = rospy.get_param("~box_width")
		self.box_height = rospy.get_param("~box_height")
		self.step_size = rospy.get_param("~step_size")
		self.cloud = self.generate_point_cloud()

	def generate_point_cloud(self):
		cloud = PointCloud()
		cloud.header.frame_id = rospy.get_param("~frame_id")
		for i in xrange(0, math.ceil((self.box_height/self.step_size) + 1)):
			cloud.points.append(Point32(i*self.step_size, 0, 0))
			cloud.points.append(Point32(i*self.step_size, self.box_width, 0))
		for i in xrange(0, math.ceil((self.box_width/self.step_size) + 1)):
			cloud.points.append(Point32(0, i*self.step_size, 0))
			cloud.points.append(Point32(self.box_height, i*self.step_size, 0))
		return cloud

if __name__ == "__main__":
	rospy.init_node("bounding_box_generator", anonymous=True)
	bounding_box_gen = BoundingBoxGenerator()
	pub = rospy.Publisher("bounding_box", PointCloud)
	r = rospy.Rate(rospy.get_param("~publishing_rate"))
	while not rospy.is_shutdown():
		cloud = bounding_box_gen.cloud
		cloud.header.stamp = rospy.Time.now()
		pub.publish(cloud)
		r.sleep()
	 
