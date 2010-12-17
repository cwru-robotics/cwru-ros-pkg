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
from cwru_base.msg import cRIOSensors

class EKF:    
  def __init__(self, gX):
    if gX is None:
      # Read from YAML
      # Kalman process noise
      qxy = float(rospy.get_param('/cwru_ekf/CWRU_EKF/qXY', '1e-10'))
      qth = float(rospy.get_param('/cwru_ekf/CWRU_EKF/qTh', '.1e-10'))
      qv = float(rospy.get_param('/cwru_ekf/CWRU_EKF/qV', '1.10579509e+00'))
      qw = float(rospy.get_param('/cwru_ekf/CWRU_EKF/qW', '1.50573452e+00'))
      qb = float(rospy.get_param('/cwru_ekf/CWRU_EKF/qBias', '1.14688434e-06'))
      q = array((qxy, qxy, qth, qv, qw, qb))
      self.Q = diagflat(q)
      
      # Model constants
      self.lec = float(rospy.get_param('/cwru_ekf/CWRU_EKF/lec', '8.57812704e-04'))
      self.rec = float(rospy.get_param('/cwru_ekf/CWRU_EKF/rec', '8.53890657e-04'))
      self.lmec = float(rospy.get_param('/cwru_ekf/CWRU_EKF/lmec', '0.000057187'))
      self.rmec = float(rospy.get_param('/cwru_ekf/CWRU_EKF/rmec', '0.000056932'))
      self.yc = float(rospy.get_param('/cwru_ekf/CWRU_EKF/yc', '1.25301732e-02'))
      self.track = float(rospy.get_param('/cwru_ekf/CWRU_EKF/track', '0.561975'))
      self.dt = float(rospy.get_param('/cwru_ekf/CWRU_EKF/dt', '0.02'))
      
      encVc = float(rospy.get_param('/cwru_ekf/CWRU_EKF/encVc', '8.79895355e-06'))
      encVl = float(rospy.get_param('/cwru_ekf/CWRU_EKF/encVl', '0.0'))
      encVs = float(rospy.get_param('/cwru_ekf/CWRU_EKF/encVs', '0.0'))
      yawVc = float(rospy.get_param('/cwru_ekf/CWRU_EKF/yawVc', '1.15810749e-02'))
      yawVl = float(rospy.get_param('/cwru_ekf/CWRU_EKF/yawVl', '0.0'))
      yawVs = float(rospy.get_param('/cwru_ekf/CWRU_EKF/yawVs', '0.0'))
      
      self.runEnc = bool(rospy.get_param('/cwru_ekf/CWRU_EKF/runEnc', 'true'))
      self.runYaw = bool(rospy.get_param('/cwru_ekf/CWRU_EKF/runYaw', 'true'))
      
      # Sensor variances
      self.encV = (encVc, encVl, encVs) # Constant, "linear", "square"
      self.yawV = (yawVc, yawVl, yawVs) # Constant, "linear", "square"
      
      self.publish = 1
    else:
      # gX is: qV qW qB encVc encVl yawVc yawVl lec rec lmec rmec yc track
    
      qV = gX[0]
      qW = gX[1]
      qB = gX[2]
      
      encVc = gX[3]
      encVl = gX[4]
      yawVc = gX[5]
      yawVl = gX[6]
      
      self.lec = gX[7]
      self.rec = gX[8]
      self.lmec = gX[9]
      self.rmec = gX[10]
      self.yc = gX[11]
      self.track = gX[12]

      # Kalman process noise
      q = array((1e-8, 1e-8, 1e-8, qV, qW, qB))
      self.Q = diagflat(q)
      
      # Sensor variances
      self.encV = (encVc, encVl, 0.0) # Constant, "linear", "square"
      self.yawV = (yawVc, yawVl, 0.0) # Constant, "linear", "square"

      self.runEnc = True
      self.runYaw = True
      
      # Model constants
      self.dt = 1.0/50.0
      
      self.publish = 0

    self.initCommon()

  def initCommon(self):
    # Kalman state variables
    self.x = zeros((6,1)) # Kalman state [x; y; th; v; w; Vbias]
    self.P = eye(6) # Covariance matrix
    
    self.firstRun = 1
    self.firstOdom = 1
    self.trans = 0
    
    if(self.publish == 1):
      rospy.init_node('cwru_ekf')
      rospy.Subscriber('crio_sensors', cRIOSensors, self.update_filter)
      self.odom_pub = rospy.Publisher('odom_ekf', Odometry)
      self.tf_br = tf.TransformBroadcaster()
      self.listener = tf.TransformListener()
      rospy.spin()
	      
  def update_filter(self, msg):
    self.modelJacobian()
    self.prediction(msg)
    
    self.sensor_update(msg)
    
    if(self.publish == 1):
      self.publish_filter()
      
    
  def prediction(self, msg):
    # Model prediction
    x = self.x[0]
    y = self.x[1]
    th = self.x[2]
    v = self.x[3]
    w = self.x[4]
    Vb = self.x[5]
          
    self.x[0] = x + v*self.dt*cos(th)
    self.x[1] = y + v*self.dt*sin(th)
    self.x[2] = th + w*self.dt
    
    # Covariance prediction
    self.P = dot(dot(self.A,self.P),self.A.T) + self.Q
    self.P[0,5] = 0.0
    self.P[5,0] = 0.0
    self.P[1,5] = 0.0
    self.P[5,1] = 0.0
    self.P[2,5] = 0.0
    self.P[5,2] = 0.0
    
  def modelJacobian(self):
    th = self.x[2]
    v = self.x[3]
    dt = self.dt
    
    self.A = eye(6)
    self.A[0,2] = -v*dt*sin(th)
    self.A[0,3] = dt*cos(th)
    self.A[1,2] = v*dt*cos(th)
    self.A[1,3] = dt*sin(th)
    self.A[2,4] = dt
    
  def sensor_update(self, msg):
    self.sensorJacobian()
    
    if(self.firstRun == 1):
      # Just record encoder values
      self.le = msg.left_wheel_encoder
      self.re = msg.right_wheel_encoder
      self.firstRun = 0
    else:
      
      # For Enc
      dle = self.lec*(msg.left_wheel_encoder-self.le)
      dre = self.rec*(msg.right_wheel_encoder-self.re)
      
      #venc = (dle + dre)/(2.0*self.dt)
      #wenc = (dle - dre)/(self.track*self.dt)
      
      yv = (msg.yaw_rate/1000.0)/self.yc*3.1415/180.0
      
      z = array([dle, dre, yv]).T
      
      self.le = msg.left_wheel_encoder
      self.re = msg.right_wheel_encoder
      
      
      lev = self.variance_from_measurement(dle, self.encV[0], self.encV[1], self.encV[2])
      rev = self.variance_from_measurement(dre, self.encV[0], self.encV[1], self.encV[2])
      yawVar = self.variance_from_measurement(yv, self.yawV[0], self.yawV[1], self.yawV[2])
      
      yc = z - self.sensor_prediction()
      
      #print "w: %s   enc: %s    yaw: %s" % (self.x[4], wenc, yv)
      
      #print "w: %s   b: %s    yaw: %s" % (self.x[4], self.x[5], yv)
      
      if(self.runEnc):
	R = diagflat(array([lev, rev]))
	y = yc[:,0:2]
	S = dot(dot(self.Henc,self.P),self.Henc.T) + R
	K = dot(dot(self.P,self.Henc.T),linalg.pinv(S))
	self.x = self.x + dot(K,y.T)
	self.P = dot(eye(6)-dot(K,self.Henc),self.P)
      
      if(self.runYaw):
	R = diagflat(array([yawVar]))
	y = yc[0,2]
	S = dot(dot(self.Hyaw,self.P),self.Hyaw.T) + R
	K = dot(dot(self.P,self.Hyaw.T),linalg.pinv(S))
	self.x = self.x + dot(K,y.T)
	self.P = dot(eye(6)-dot(K,self.Hyaw),self.P)
      
    
  def sensor_prediction(self):
    v = self.x[3]
    w = self.x[4]
    b = self.x[5]
    y = w + b#- 0.041534392
    le = 1.0/2.0*self.dt*(2.0*v+self.track*w)
    re = 1.0/2.0*self.dt*(2.0*v-self.track*w)
    z = array([le, re, y])
    return z.T
    
  @staticmethod
  def variance_from_measurement(meas, c0, c1, c2):
    return c0 + c1 * abs(meas) + c2 * pow(meas,2)
    
  def sensorJacobian(self):
    dt = self.dt
    self.Henc = zeros((2,6))
    self.Henc[0,3] = dt
    self.Henc[1,3] = dt
    self.Henc[0,4] = self.track*dt/2.0
    self.Henc[1,4] = -self.track*dt/2.0
    self.Hyaw = zeros((1,6))
    self.Hyaw[0,4] = 1.0
    self.Hyaw[0,5] = 1.0
    
  def getState(self):
    return self.x
    
  def getCovariance(self):
    return self.P
    
  def publish_filter(self):    
    if(self.trans == 0):
      try:
	(self.trans,self.rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
	(trans2,rot2) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	self.tf_br.sendTransform(translation = self.trans,
		  rotation = self.rot,
		  time = rospy.Time.now(),
		  child = 'odom_ekf',
		  parent = 'map')
	self.x[2] -= tf.transformations.euler_from_quaternion(rot2)[2]
	self.x[0] += trans2[0]
	self.x[1] -= trans2[1]
      except (tf.LookupException, tf.ConnectivityException):
	a = 0
    else: # Copy the transform from amcl to odom
      self.tf_br.sendTransform(translation = self.trans, 
		rotation = self.rot,
		time = rospy.Time.now(),
		child = 'odom_ekf',
		parent = 'map')
		
    x = self.x[0]
    y = -self.x[1] # Change to ROS
    th = -self.x[2] # Change to ROS
    v = self.x[3]
    w = -self.x[4] # Change to ROS
    
    current_time = rospy.Time.now()
    quaternion = tf.transformations.quaternion_about_axis(th, (0,0,1))
    self.tf_br.sendTransform(translation = (x, y, 0), 
		    rotation = tuple(quaternion),
		    time = current_time,
		    child = 'base_link_ekf',
		    parent = 'odom_ekf')
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = 'odom_ekf'

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = Quaternion(*quaternion)

    odom_msg.pose.covariance[0] = self.P[0,0]
    odom_msg.pose.covariance[7] = self.P[1,1]
    odom_msg.pose.covariance[35] = self.P[2,2]

    odom_msg.child_frame_id = 'base_link_ekf'
    odom_msg.twist.twist.linear.x = v
    odom_msg.twist.twist.angular.z = w

    odom_msg.twist.covariance[0] = self.P[3,3]
    odom_msg.twist.covariance[35] = self.P[4,4]

    self.odom_pub.publish(odom_msg)


if __name__ == '__main__':
	try:
	  myEKF = EKF(None)

	except rospy.ROSInterruptException: pass