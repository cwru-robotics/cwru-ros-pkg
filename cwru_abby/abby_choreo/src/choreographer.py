#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""Goal choreographer based on the tour executive for Roberto.
Orignal tour executive written by Jesse Fish, Toby Waite, and Bill Kulp.
Modified for use on the robot Abby by Edward Venator."""

import roslib
roslib.load_manifest('abby_choreo')
import rospy

import time
import sys

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


def main(filename):
  
  #open the yaml config file and load path
  with open(filename, 'r') as datafile:
    data = yaml.safe_load(datafile)
  goallist=data['goals']
  for entry in goallist:
    rospy.logdebug(entry)
  iterator=goallist.__iter__();
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  while(not client.wait_for_server(rospy.Duration.from_sec(5.0)):
    rospy.loginfo("Waiting for movebase server");
  
  while(1):
    try:
      rospy.logInfo('Loading next goal.')
      nextThing = iterator.next()
      goal = create_move_base_goal_from_yaml(nextThing['goal'])
      rospy.logdebug('Sending goal.')
      client.send_goal(goal)
      #client.wait_for_result()
      rospy.logdebug('got next file')
      
      while(1):
        if client.get_state() == GoalStatus.SUCCEEDED :
          rospy.loginfo('Goal succeeded.')
          break
        elif client.get_state()==GoalStatus.ABORTED :
          rospy.logwarn('Goal aborted. Resending goal.')
          client.send_goal(goal)
        elif client.get_state()==GoalStatus.REJECTED :
          rospy.logwarn('Goal rejected. Resending goal.')
          client.send_goal(goal)
        else:
          time.sleep(.7)
    except StopIteration:
      break;
  rospy.sleep(2)


if __name__ == '__main__':
    rospy.init_node('abby_choreographer')
    main(rospy.get_param('~filename'))

