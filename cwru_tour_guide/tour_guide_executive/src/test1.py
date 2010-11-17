#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy


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

sound_prefix=''

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


#define an empty stall function for now
def stall():  
  return 'testsound1.wav'

def main(filename):
  soundpath=rospy.get_param('~soundpath')
  
  #open the yaml config file and load path and tour
  with open(filename, 'r') as datafile:
    data = yaml.safe_load(datafile)
  jokelist=data['jokes']
  goallist=data['goals']
  for entry in goallist:
    print entry
  p=0
  i=vlc.Instance()
  iterator=goallist.__iter__();
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()
  while(1):
    try:
      print 'starting location'
      nextThing=iterator.next()
      goal=create_move_base_goal_from_yaml(nextThing['goal'])
      sound=nextThing['wavs']
      print 'send location'
      client.send_goal(goal)
      #client.wait_for_result()
      print 'got next file'
      if p!=0:
        p.release()

      p=vlc.MediaPlayer(sound_prefix + sound[0])
      print 'created next file'
      
      print 'retained next file'
      p.play()
      print(p.get_state())
      
      while(1):
 #       print 'starting inner loop'
        while(p.get_state()!=vlc.State.Ended):
 #         print(p.get_state())
          time.sleep(.2)

        time.sleep(.7)

        print 'finished sound'
        if client.get_state() == GoalStatus.SUCCEEDED :
          break
        elif client.get_state()==GoalStatus.ABORTED or client.get_state()==GoalStatus.REJECTED :
          #resend the goal
          client.send_goal(goal)
        else:
 #         print 'starting stall sound'
          time.sleep(.7)
       #   p=vlc.MediaPlayer(sound_prefix + stall())
       #   p.play()
        #if client.get_state() == GoalStatus.SUCCEEDED:
	    #rospy.loginfo("Goal executed successfully")
	#else:
	    #rospy.logerr("Could not execute goal for some reason")
      
    except StopIteration:
      break;

  rospy.sleep(2)

  #time.sleep(5);

if __name__ == '__main__':
  rospy.init_node('roberto_goal_planner')
  sound_prefix=rospy.get_param('~soundpath')
  print('woooow'+sound_prefix)
  
  main(rospy.get_param('~filename'))
  
