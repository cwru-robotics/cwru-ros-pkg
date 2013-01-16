#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy

import copy

from geometry_msgs.msg import Twist

def slow_spin():
    rospy.loginfo('Press enter to start spinning at .5 rad/sec. Press ctrl-c to stop and kill this node.')
    
    cmdPub = rospy.Publisher("cmd_vel", Twist)
    rospy.init_node('slow_spin')
    
    twistMsg = Twist()
    
    twistMsg.angular.x = 0
    twistMsg.angular.y = 0
    twistMsg.angular.z = 0.5
    
    twistMsg.linear.x = 0
    twistMsg.linear.y = 0
    twistMsg.linear.z = 0
    trash = raw_input()

    try:
        while not rospy.is_shutdown():
            rospy.sleep(.05)
            cmdPub.publish(twistMsg)
    except KeyboardInterrupt:
        pass
    twistMsg.angular.z = 0
    cmdPub.publish(twistMsg)
    rospy.loginfo('Stopped spinning. Node is shutting down.')
if __name__ == '__main__':
    slow_spin()
