#! /usr/bin/env python
import roslib; roslib.load_manifest('tour_guide_kinect')
import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
	rospy.loginfo('I heard something!')

def listner():
	rospy.init_node('kinect_listner', anonymous=True)
	rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
	rospy.spin()

if __name__=='__main__':
	listner()
