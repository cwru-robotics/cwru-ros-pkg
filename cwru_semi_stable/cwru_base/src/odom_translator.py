#!/usr/bin/env python
import roslib
roslib.load_manifest('cwru_base')
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from cwru_base.msg import Pose

odom_pub = rospy.Publisher('odom', Odometry)
tf_br = tf.TransformBroadcaster()

def changePoseCoordFrame(pose):
	p = Pose(x = pose.x, y = -1*pose.y, theta = -1*pose.theta, x_var = pose.x_var, y_var = pose.y_var, theta_var = pose.theta_var, vel = pose.vel, omega = -1*pose.omega, vel_var = pose.vel_var, omega_var = pose.omega_var)
	return p

def handle_harlie_pose(msg):
	pose = changePoseCoordFrame(msg)
	current_time = rospy.Time.now()
	quaternion = tf.transformations.quaternion_about_axis(pose.theta, (0,0,1))
	tf_br.sendTransform(translation = (pose.x, pose.y, 0), 
			rotation = tuple(quaternion),
			time = current_time,
			child = 'base_link',
			parent = 'odom')
	odom_msg = Odometry()
	odom_msg.header.stamp = current_time
	odom_msg.header.frame_id = 'odom'

	odom_msg.pose.pose.position.x = pose.x
	odom_msg.pose.pose.position.y = pose.y
	odom_msg.pose.pose.position.z = 0.0
	odom_msg.pose.pose.orientation = Quaternion(*quaternion)

	odom_msg.pose.covariance[0] = pose.x_var
	odom_msg.pose.covariance[7] = pose.y_var
	odom_msg.pose.covariance[35] = pose.theta_var

	odom_msg.child_frame_id = 'base_link'
	odom_msg.twist.twist.linear.x = pose.vel
	odom_msg.twist.twist.angular.z = pose.omega

	#TODO Here starts the constant multiplier noise. Needs real variance...
	odom_msg.twist.covariance[0] = pose.vel_var
	odom_msg.twist.covariance[35] = pose.omega_var

	odom_pub.publish(odom_msg)

if __name__ == "__main__":
	rospy.init_node('harlie_odom_translator')
	rospy.Subscriber('pose', Pose, handle_harlie_pose)
	rospy.spin()
