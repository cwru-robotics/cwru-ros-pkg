# -*- coding: utf-8 -*-
#! /usr/bin/env python

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy

import tf

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
  initialPose=PoseWithCovarianceStamped()
  initialPose.header.frame_id='/map'
  initialPose.pose.pose.position.x=0
  initialPose.pose.pose.position.y=0
  
  quaternion = tf.transformations.quaternion_about_axis(0 , (0,0,1))
  initialPose.pose.pose.orientation= Quaternion(*quaternion)
  initialPose.pose.covariance[0]=.25
  initialPose.pose.covariance[7]=.25
  initialPose.pose.covariance[35]=.0685
  pub = rospy.Publisher('InitialPose', PoseWithCovarianceStamped)

  pub.publish(initialPose)

if __name__ == '__main__':
  rospy.init_node('roberto_intial_pose_seeder')
  main()
  