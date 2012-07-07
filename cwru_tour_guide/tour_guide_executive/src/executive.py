#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import vlc
import yaml
import math

import roslib
roslib.load_manifest('tour_guide_executive')
import rospy

import cv
import tf
import actionlib

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Odometry

rospy.init_node('roberto_goal_planner')

# Create a transform listener to transform to map coordinates
listener       = tf.TransformListener()
last_odom      = PoseStamped()
last_odom_time = rospy.Time.from_sec(0)


# Odometry callback
def odom_callback(odom):
    # Extract pose information
    odom_pose        = PoseStamped()
    odom_pose.header = odom.header
    odom_pose.pose   = odom.pose.pose

    global last_odom
    global last_odom_time

    now = rospy.Time.now()
 
    # Transform to map coordinates
    try:    
      transformed    = listener.transformPose('map', odom_pose)
      last_odom      = transformed
      last_odom_time = now
         
    except tf.Exception:
      rospy.logwarn('TF exception converting /odom to /map')


# Using information from odometry, tells whether we are within in a distance of the goal
def dist_to_goal(goal):
    global last_odom
    global last_odom_time

    # Make sure we have current odometry from the robot
    now = rospy.Time.now()

    if now < last_odom_time:
      rospy.logerror('WTF time travel')
      return 999

    if now - last_odom_time > rospy.Duration.from_sec(2.0):
      rospy.loginfo('stale odom: ' + str( (now-last_odom_time).to_sec() ) + ' secs old')
      return 999

    dx = goal.pose.position.x - last_odom.pose.position.x
    dy = goal.pose.position.y - last_odom.pose.position.y
    dist = math.sqrt(dx*dx + dy*dy)

    return dist


def create_move_base_goal_from_yaml(yaml_goal):
    """Creates a MoveBaseGoal from a goal loaded up from the yaml file"""
    goal = PoseStamped()
    goal.header.frame_id = '/map'
    goal.header.stamp    = rospy.Time.now()
    
    goal.pose.position.x = yaml_goal['x']
    goal.pose.position.y = yaml_goal['y']
    quaternion = tf.transformations.quaternion_about_axis(yaml_goal['theta'], (0,0,1))
    goal.pose.orientation = Quaternion(*quaternion)
    
    return goal


#define an empty stall function for now
def stall():  
  return 'testsound1.wav'


def main():
  skip_sounds = False

  # Load parameter files
  filename     = rospy.get_param('~filename' )
  sound_prefix = rospy.get_param('~soundpath')

  rospy.loginfo('Running tour script   \'' + filename     + '\'')
  rospy.loginfo('Looking for sounds in \'' + sound_prefix + '\'')
  
  #open the yaml config file and load path and tour
  with open(filename, 'r') as datafile:
    data = yaml.safe_load(datafile)
  jokelist=data['jokes']
  goallist=data['goals']
  iterator=goallist.__iter__();

  # Set up VLC player
  player=0
  instance=vlc.Instance()
  
  # Our interface to the planner
  pub = rospy.Publisher('/goal', PoseStamped)
  odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
  
  while not rospy.is_shutdown():
    try:
      # Get the next goal and sound
      rospy.loginfo('Getting next goal')
      nextThing=iterator.next()
      goal = create_move_base_goal_from_yaml(nextThing['goal'])
      sound=nextThing['wavs']

      if skip_sounds == True:
        sound = 'blank_sound.ogg'
      
      imgname = str(sound_prefix + nextThing['pics'][0])
      img = cv.LoadImage(imgname)
      cv.NamedWindow("window")
      cv.ShowImage("window", img)
      cv.WaitKey(100)

      # Play the sound and display the image
      rospy.loginfo('Playing %s' %(sound[0]))
      if player!=0:
        player.release()
      player=vlc.MediaPlayer(sound_prefix + sound[0])
      player.play()
      
      # Wait for sound to finish
      while(player.get_state()!=vlc.State.Ended):
        time.sleep(.2)
      
      # Send goal to planner
      rospy.loginfo('Sending goal to planner: %s' %goal)
      pub.publish(goal)
      
      # Wait for planner to get to goal
      dist = dist_to_goal(goal)

      while dist > 1.0 and not rospy.is_shutdown():
        rospy.loginfo('Distance to goal: ' + str(dist) + 'm')
        time.sleep(0.5)
        dist = dist_to_goal(goal)
      
      cv.DestroyAllWindows()
      cv.WaitKey(100)
      time.sleep(5)
    except StopIteration:
      break;
    
if __name__ == '__main__':
  main()
  
