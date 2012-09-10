#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('irc5')
import rospy

import copy

from geometry_msgs.msg import Twist

def twist_ramp_linear(v_min, v_max, accel):
    rospy.loginfo('Generating a ramp from %f m/s to %f m/s at %f m/s^2',v_min,v_max, accel)
    t_step = .05
    
    cmdPub = rospy.Publisher("cmd_vel", Twist)
    rospy.init_node('twist_ramp')
    
    twistMsg = Twist()
    
    twistMsg.angular.x = 0
    twistMsg.angular.y = 0
    twistMsg.angular.z = 0
    
    twistMsg.linear.x = 0
    twistMsg.linear.z = 0
    
    if accel > 0:
        twistMsg.linear.y = v_min
    else:
        twistMsg.linear.y = v_max
    wallTime = rospy.get_time()
    while not rospy.is_shutdown() and twistMsg.linear.y <= v_max and twistMsg.linear.y >= v_min:
        rospy.sleep(.05)
        oldWallTime = wallTime
        wallTime = rospy.get_time()
        timeStep = wallTime - oldWallTime
        cmdPub.publish(twistMsg)
        twistMsg.linear.y = twistMsg.linear.y + accel * timeStep
    twistMsg.linear.y = 0
    cmdPub.publish(twistMsg)
if __name__ == '__main__':
'''    try:
      min = rospy.get_param("~min");
    except KeyError:
      print "Parameter min not set";
    try:
      max = rospy.get_param("~max");
    except KeyError:
      print "Parameter max not set";
    try:
      accel = rospy.get_param("~accel");
    except KeyError:
      print "Parameter accel not set";'''
    
    try:
        twist_ramp_linear(min, max, accel)
    except NameError:
        print "Parameters not set. Required parameters are min, max, and accel"
