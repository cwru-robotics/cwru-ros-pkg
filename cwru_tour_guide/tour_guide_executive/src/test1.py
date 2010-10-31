#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy

import sys
#import vlc

import tf
import actionlib

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import yaml

def main(filename):
  soundpath=rospy.get_param('~soundpath')
  
  #open the yaml config file and load path and tour
  with open(filename, 'r') as datafile:
    data = yaml.safe_load(datafile)

  goallist=data['goals']
  for entry in goallist:
    print entry
    
  iterator=goallist.__iter__();
  
  
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = '/map'
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = 21.8202071179
  goal.target_pose.pose.position.y = 5.0818858318
  quaternion = tf.transformations.quaternion_about_axis(0, (0,0,1))
  goal.target_pose.pose.orientation = Quaternion(*quaternion)
  
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()

  client.send_goal(goal)
  client.wait_for_result()
  if client.get_state() == GoalStatus.SUCCEEDED:
      rospy.loginfo("Goal executed successfully")
  else:
      rospy.logerr("Could not execute goal for some reason")
      
  #i=vlc.Instance()
  #p=vlc.MediaPlayer('testsound1.wav')
  #p.play()

  #time.sleep(5);

if __name__ == '__main__':
  rospy.init_node('roberto_goal_planner')
  main(rospy.get_param('~filename'))
  
