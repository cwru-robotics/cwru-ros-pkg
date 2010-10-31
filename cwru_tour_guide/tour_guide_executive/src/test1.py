# -*- coding: utf-8 -*-
#! /usr/bin/env python

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy

import sys
import vlc

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

  for entry in goallist:
    print entry
    
  iterator=goallist.__iter__();


  #i=vlc.Instance()
  #p=vlc.MediaPlayer('testsound1.wav')
  #p.play()

  #time.sleep(5);

if __name__ == '__main__':
  rospy.init_node('roberto_goal_planner')
  main(rospy.get_param('~filename'))
  