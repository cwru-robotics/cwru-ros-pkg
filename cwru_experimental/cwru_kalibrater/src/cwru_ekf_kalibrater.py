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
#from cwru_ekf_dynamic import EKF
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import fmin_powell

import sys
import rosbag
import pylab

class KF_Tuner:
  def __init__(self, bagPath):
    # gX is: qV qW qB encVc encVl yawVc yawVl lec rec lmec rmec yc track
    self.bagsf = []
    #self.bagsf.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/straight.bag"))
    #self.bagsf.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/fishtail.bag"))
    #self.bagsf.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/loop.bag"))
    #self.bagsf.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/u.bag"))
    #self.bagsf.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/l.bag"))
    #self.bags.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/spin.bag"))
    #self.bags.append(rosbag.Bag("/home/chad/cwru-ros-pkg/cwru_experimental/cwru_kalibrater/src/logs/spin2.bag"))
    #self.names = []
    #self.names.append("straight")
    #self.names.append("fishtail")
    #self.names.append("loop")
    #self.names.append("u")
    #self.names.append("l")
    '''for i in range(len(self.bagsf)):
      self.bags = []
      for j in range(len(self.bagsf)):
	#if(i != j):
	self.bags.append(self.bagsf[i])'''
    self.bagsf.append(rosbag.Bag(bagPath))
    self.bags = self.bagsf
    self.tune = (True, True, True, True, False, True, False, True, True, False, False, True, False)
    self.default = [1.0, 1.0, 1e-6, .01, 0.0, .01, 0.0, 0.00087589, 0.00087196, 0.000058393, 0.000058131, 0.0126276, 0.561975]
    glX = (1.0, 1.0, 1e-6, .01, .01, 0.00087589, 0.00087196, 0.0126276)
    gXopt = fmin(self.newEKF, glX)
    print "----------------------------------"
    print gXopt
    self.bags = [self.bagsf[i]]
    print self.newEKF(gXopt)
    for bag in self.bagsf:
      bag.close()
    
  def newEKF(self, glX):
    print glX
    gX = self.default
    j = 0
    for i in range(len(gX)):
      if(self.tune[i]):
	gX[i] = glX[j]
	j = j + 1
    

    rmsSum = 0.0
    pSum = 0.0
    T = 0
    C = 0

    for b in range(len(self.bags)):
      curBag = self.bags[b]
      map2odomtrans = False
      map2odomrot = False
      odom2basetrans = False
      odom2baserot = False
      odomPos = False
      amclPos = False
      amclRot = False
      amclInitPos = False
      amclInitRot = False
      state = zeros((6,1))
      state[0] = False
      #print gX
      thisEKF = EKF(gX)
      actualList = []
      ekfList = []
      for topic, msg, t in curBag.read_messages(topics=['/odom', '/crio_sensors', '/tf']):
	if(topic == "/odom"):
	  odomPos = msg.pose.pose.position
	  if(odom2basetrans == False):
	    odom2basetrans = msg.pose.pose.position
	    quat = msg.pose.pose.orientation
	    quat = [quat.x, quat.y, quat.z, quat.w]
	    odom2baserot = tf.transformations.euler_from_quaternion(quat)[2]
	elif(topic == "/tf"):
	  tfStamp = msg.transforms[0]
	  frameid = tfStamp.header.frame_id
	  childid = tfStamp.child_frame_id
	  if(frameid == '/map' and childid == "/odom"): # This is an AMCL update grab it!
	    amclPos = tfStamp.transform.translation
	    quat = tfStamp.transform.rotation
	    quat = [quat.x, quat.y, quat.z, quat.w]
	    amclRot = tf.transformations.euler_from_quaternion(quat)[2]
	    if(amclInitPos == False):
	      amclInitPos = amclPos
	      amclInitRot = amclRot
	elif(topic == "/crio_sensors"):
	  # Run EKF update and add to RMS
	  thisEKF.update_filter(msg)
	  state = thisEKF.getState()
	  covar = thisEKF.getCovariance()
	  # Figure out actual position
	  if(odom2basetrans != False and odomPos != False and state[0] != False and amclPos != False):
	    # Transform odom to map to get actual
	    actual = array(self.transformFrame(odomPos.x, odomPos.y, amclPos, amclRot))
	    #print actual
	    
	    # Transform EKF into map to get estimated
	    kfOdom = self.transformFrame(state[0], -state[1], odom2basetrans, odom2baserot)
	    kfMap = array(self.transformFrame(kfOdom[0], kfOdom[1], amclInitPos, amclInitRot))
	    
	    #print kfMap
	    actualList.append([actual[0],actual[1]])
	    ekfList.append([kfMap[0],kfMap[1]])
	    
	    try:
	      rmsSum += math.pow(actual[0]-kfMap[0],2) + math.pow(actual[1]-kfMap[1],2)
	      H = zeros((2,6))
	      H[0,0] = 1.0
	      H[1,1] = 1.0
	      omega = dot(H,dot(covar,H.T))
	      Ps = log(2.0*3.14159*omega)+dot(dot((actual-kfMap).T,linalg.pinv(omega)),(actual-kfMap))
	      pSum = (Ps[0,0] + Ps[1,1])
	      T += 1
	    except:
	      rmsSum += 1e6
	      pSum += 1e6
	      T += 1
	      
      try:
	# Plot
	actualList = array(actualList)
	ekfList = array(ekfList)
	pylab.plot(actualList[:,0], actualList[:,1], 'g', ekfList[:,0], ekfList[:,1], 'b')
	pylab.xlabel('x coord (m)')
	pylab.ylabel('y coord (m)')
	pylab.savefig('currentPaths'+self.names[b])
	pylab.clf()
      except:
	pylab.clf()
	    
    rmsErr = math.sqrt(1.0/float(T)*rmsSum)
    print rmsErr
    #print pSum
    
    #raw_input()
    return rmsErr
  
  def transformFrame(self, x, y, trans, th):
    xp = x*cos(th)-y*sin(th)+trans.x
    yp = x*sin(th)+y*cos(th)+trans.y
    return [xp, yp]
    
  '''def logsig(self,x):
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
    return 2.0/(1.0+exp(-2.0*x))-1.0'''

if __name__ == '__main__':
	try:
	  try:
	    bagPath = sys.argv[1]
	  except IndexError:
	    sys.exit("ERROR: You did not specify the bag location!")
	  try:
	    EKFFile = sys.argv[2].split(".py")[0]
	    im_string = "from " + EKFFile + " import EKF"
	    exec im_string
	  except IndexError:
	    sys.exit("ERROR: You did not specify which EKF to run!")
	  myKFT = KF_Tuner("")
	except rospy.ROSInterruptException: pass