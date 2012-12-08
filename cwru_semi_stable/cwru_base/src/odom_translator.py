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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from cwru_base.msg import Pose
from math import pi

odom_pub = rospy.Publisher('odom', Odometry)
tf_br = tf.TransformBroadcaster()

def changePoseCoordFrame(pose):
	p = Pose(x = pose.x, y = -1*pose.y, theta = -1*pose.theta, x_var = pose.x_var, y_var = pose.y_var, theta_var = pose.theta_var, vel = pose.vel, omega = -1*pose.omega, vel_var = pose.vel_var, omega_var = pose.omega_var)
	return p

def handle_harlie_pose(msg, push_casters):
	pose = changePoseCoordFrame(msg)
	current_time = rospy.Time.now()
	theta = pose.theta + pi if push_casters else pose.theta
	quaternion = tf.transformations.quaternion_about_axis(theta, (0,0,1))
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

	odom_msg.twist.covariance[0] = pose.vel_var
	odom_msg.twist.covariance[35] = pose.omega_var

	odom_pub.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node('harlie_odom_translator')
    push_casters = rospy.get_param("~push_casters")
    rospy.Subscriber('pose', Pose, handle_harlie_pose, push_casters)
    rospy.spin()
