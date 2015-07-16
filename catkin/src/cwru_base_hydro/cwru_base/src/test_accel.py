#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('irc5')
import rospy

import copy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

v_min = .1
v_max = 1
accel = 4

at_speed = False


def velocity_callback(odom):
    global wallTime
    global at_speed
    if not at_speed:
        if odom.twist.twist.x >= v_max:
            delta_t = rospy.get_time() - wallTime
            rospy.loginfo('Reached speed in %f seconds (%f m/s^2)',delta_t, v_max/delta_t)
            at_speed = True
    else:
        rsopy.loginfo('Speed: %f')

rospy.loginfo('Accelerating to max velocity at %f m/s^2 and recording actual velocity', accel)
t_step = .05

cmdPub = rospy.Publisher("cmd_vel", Twist)
velSub = rospy.Subscriber("odom", Odometry, velocity_callback)
rospy.init_node('twist_ramp')

twistMsg = Twist()

twistMsg.angular.x = 0
twistMsg.angular.y = 0
twistMsg.angular.z = 0

twistMsg.linear.y = 0
twistMsg.linear.z = 0

if accel > 0:
    twistMsg.linear.x = v_min
else:
    twistMsg.linear.x = v_max
wallTime = rospy.get_time()
while not rospy.is_shutdown() and twistMsg.linear.x <= v_max and twistMsg.linear.x >= v_min:
    rospy.sleep(.05)
    oldWallTime = wallTime
    wallTime = rospy.get_time()
    timeStep = wallTime - oldWallTime
    cmdPub.publish(twistMsg)
    twistMsg.linear.x = twistMsg.linear.x + accel * timeStep
rospy.sleep(1.0)
twistMsg.linear.x = 0
cmdPub.publish(twistMsg)
