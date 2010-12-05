#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2010, Chad Rockey
# All rights reserved
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

from numpy import *
import roslib
roslib.load_manifest('cwru_kalibrater')
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
from cwru_base.msg import cRIOSensors
from cwru_ekf_thrun2 import EKF
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import fmin_powell

import sys
import rosbag

class KF_Tuner:
  def __init__(self, bagPath):
    # gX is: qV qW qB encVc encVl yawVc yawVl lec rec yc track
    self.bag = rosbag.Bag(bagPath)
    #self.mag = (20, 20, 1, 1, 1, 1, 1)#, 1e-5, 1e-5, .0126, .56)
    #self.signed = (1, 1, 1, 1, -1, 1, -1)#, 1e-5, 1e-5, .0126, .56)
    #gX = (1.0, 1.0, 1e-6, .002, .002, .002, .002)#, 1e-5, 1e-5, .0126, .56)
    glX = (1, 1, 1e-6, 0.01)#(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)#, 1e-5, 1e-5, .0126, .56)
    gXopt = fmin(self.newEKF, glX, xtol=1e-6)#fmin_bfgs(self.newEKF, glX)#
    self.bag.close()
    
  def newEKF(self, glX):
    gX = glX
    '''for i in range(len(glX)):
      if(self.signed[i] > 0):
	gX[i] = self.mag[i]*self.logsig(glX[i])
      else:
	gX[i] = self.mag[i]*self.signlogsig(glX[i])'''
    
    map2odomtrans = False
    map2odomrot = False
    odom2basetrans = False
    odom2baserot = False
    odomPos = False
    amclPos = False
    amclRot = False
    state = [False]
    rmsSum = 0.0
    T = 0
    print gX
    thisEKF = EKF(gX)
    for topic, msg, t in self.bag.read_messages(topics=['/odom', '/crio_sensors', '/tf']):
      if(topic == "/odom"):
	if(odom2basetrans == False):
	  odom2basetrans = msg.pose.pose.position
	  quat = msg.pose.pose.orientation
	  quat = [quat.x, quat.y, quat.z, quat.w]
	  odom2baserot = tf.transformations.euler_from_quaternion(quat)[2]
	odomPos = msg.pose.pose.position
      elif(topic == "/tf"):
	tfStamp = msg.transforms[0]
	frameid = tfStamp.header.frame_id
	childid = tfStamp.child_frame_id
	if(frameid == '/map' and childid == "/odom"): # This is an AMCL update grab it!
	  amclPos = tfStamp.transform.translation
	  quat = tfStamp.transform.rotation
	  quat = [quat.x, quat.y, quat.z, quat.w]
	  amclRot = tf.transformations.euler_from_quaternion(quat)[2]
      elif(topic == "/crio_sensors"):
	# Run EKF update and add to RMS
	thisEKF.update_filter(msg)
	state = thisEKF.getState()
	# Figure out actual position
	if(odom2basetrans != False and odomPos != False and state[0] != False and amclPos != False):
	  # Transform odom to map to get actual
	  actual = self.transformFrame(odomPos.x, odomPos.y, amclPos, amclRot)
	  
	  # Transform EKF into map to get estimated
	  kfOdom = self.transformFrame(state[0], state[1], odom2basetrans, odom2baserot)
	  kfMap = self.transformFrame(kfOdom[0], kfOdom[1], amclPos, amclRot)
	  
	  rmsSum += math.pow(actual[0]-kfMap[0],2) + math.pow(actual[1]-kfMap[1],2)
	  T += 1
    rmsErr = math.sqrt(1.0/float(T)*rmsSum)
    print rmsErr
    return rmsErr
  
  def transformFrame(self, x, y, trans, th):
    xp = x*cos(th)-y*sin(th)+trans.x
    yp = x*sin(th)+y*cos(th)+trans.y
    return [xp, yp]
    
  def logsig(self,x):
    if(x > 100):
      return 1.0
    elif(x < -100):
      return 0.0
    else:
      return 1.0 / (1.0 + exp(-x))
      
  def signlogsig(self,x):
    if(x > 100):
      return 1.0
    elif(x < -100):
      return -1.0
    else:
      return 2.0 / (1.0 + exp(-x))-1.0
    
  def tansig(self, x):
    return 2.0/(1.0+exp(-2.0*x))-1.0

if __name__ == '__main__':
	try:
	  try:
	    bagPath = sys.argv[1]
	  except IndexError:
	    sys.exit("ERROR: You did not specify the bag location!")
	  myKFT = KF_Tuner(bagPath)
	except rospy.ROSInterruptException: pass