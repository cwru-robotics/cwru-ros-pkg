#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import copy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def sim_joint_stream():
    jointPub = rospy.Publisher("command", JointTrajectory)
    rospy.init_node('sim_joint_stream')
    velocity = 10.0
    jointRange = 2*.77777770 #radians
    jointNum = 0 
    while not rospy.is_shutdown():
        jointAngles = [.0, 0., 0, 0, 0, 0] #initial arm position in radians
        jointTrajectoryMsg = JointTrajectory()
        jointTrajectoryMsg.joint_names= ["joint1","joint2","joint3","joint4","joint5","joint6"]
        for seq in range(0,10):
            jointAngles[jointNum] = jointAngles[jointNum] + (jointRange/11)
            jointTrajectoryPt = JointTrajectoryPoint()
            jointTrajectoryPt.velocities = [velocity] * 6
            jointTrajectoryPt.positions = copy.deepcopy(jointAngles)
            jointTrajectoryMsg.points.append(jointTrajectoryPt)
        for seq in range(10,0):
            jointAngles[jointNum] = jointAngles[jointNum] + (jointRange/11)
            jointTrajectoryPt = JointTrajectoryPoint()
            jointTrajectoryPt.velocities = [velocity] * 6
            jointTrajectoryPt.positions = copy.deepcopy(jointAngles)
            jointTrajectoryMsg.points.append(jointTrajectoryPt)
        jointPub.publish(jointTrajectoryMsg)
        rospy.sleep(30)

if __name__ == '__main__':
    sim_joint_stream()
