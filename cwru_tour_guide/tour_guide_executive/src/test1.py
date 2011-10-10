#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy

import math
import random
import time
import sys
import vlc

import tf
import actionlib

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import yaml

def create_move_base_goal_from_yaml(yaml_goal):
    """Creates a MoveBaseGoal from a goal loaded up from the yaml file"""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = yaml_goal['x']
    goal.target_pose.pose.position.y = yaml_goal['y']
    quaternion = tf.transformations.quaternion_about_axis(yaml_goal['theta'], (0,0,1))
    goal.target_pose.pose.orientation = Quaternion(*quaternion)
    
    return goal

def create_move_base_goal_at_random():

    MAX_X = 10
    MIN_X = 1
    MAX_Y = 10
    MIN_Y = 1

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = '/map'
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = random.uniform(MIN_X, MAX_X)
    goal.target_pose.pose.position.y = random.uniform(MIN_Y, MAX_Y)
    theta = random.uniform(0,2*math.pi)
    quaternion = tf.transformations.quaternion_about_axis(theta, (0,0,1))
    goal.target_pose.pose.orientation = Quaternion(*quaternion)

    return goal
    
def main():
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()
  while(1):
    try:
      import ipdb; ipdb.set_trace()

      print 'starting location'
      goal=create_move_base_goal_at_random()
      print 'send location'
      client.send_goal(goal)
      client.wait_for_result()
      
      while(1):
	current_state = client.get_state()
        if current_state == GoalStatus.SUCCEEDED or current_state == GoalStatus.ABORTED or current_state == GoalStatus.REJECTED :
            break
        else:
            time.sleep(.7)
            print "status: %s" % current_state
            
    except StopIteration:
      break;

  rospy.sleep(2)

if __name__ == '__main__':
  rospy.init_node('roberto_goal_planner')
  main()
