#!/usr/bin/env python

import roslib
roslib.load_manifest("harlie_vision")
import rospy
import struct

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def image_callback(img, pub):
	cloud = PointCloud()
	cloud.header.stamp = rospy.Time.now()
	cloud.header.frame_id = "/vision_cloud"
	point = Point32(300*0.05, -300*0.05, 0.1)
	for i in range(0, img.height):
		point.y = -300*0.05
		for j in range(0, img.width):
			data = img.data[img.step*i+j]
			val = struct.unpack("B", data)[0]
			if(val/255.0 > 0.1):
				cloud.points.append(Point32(point.x, point.y, point.z))
			point.y += 0.05
		point.x -= 0.05
	pub.publish(cloud)	

if __name__ == "__main__":
	rospy.init_node("image_to_point_cloud")
	pub = rospy.Publisher("vision_cloud", PointCloud) 
	rospy.Subscriber("plan_view", Image, image_callback, pub)
	rospy.spin()
