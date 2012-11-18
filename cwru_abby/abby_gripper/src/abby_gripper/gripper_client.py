#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_gripper')
import rospy
import actionlib
import sys

from chores.msg import *

if __name__ == '__main__':
    if len(sys.argv) < 1:
        raise InputError('Command Line Arguments', 'Usage: gripper_client <open/close>')
    if sys.argv[0] != 'open' and sys.argv[0] != 'close':
        raise InputError('Command Line Arguments', 'Usage: gripper_client <open/close>')
    
    rospy.init_node('gripper_client')
    client = actionlib.SimpleActionClient('gripper', ActuateGripperAction)
    client.wait_for_server()

    goal = ActuateGripperGoal()
    if sys.argv[0] == 'open':
        goal.position = 0
    else:
        goal.position = 1
    
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
