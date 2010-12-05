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
      q = array((.000001, .000001, .000001, 10, 10, .0000000001))
      self.Q = diagflat(q)
      
      # Model constants
      self.lec = 0.00087589
      self.rec = 0.00087196
      self.lmec = self.lec / 15.0
      self.rmec = self.rec / 15.0
      self.yc = 0.0126276
      self.track = 0.561975
      self.dt = 1.0/50.0
      
      # Sensor variances
      self.encV = (1e-12, 0.0002, 0.0) # Constant, "linear", "square"
      self.yawV = (.00000019, .000048345, 0.0) # Constant, "linear", "square"
      
      self.publish = 1
    else:
      # gX is: qV qW qB encVc encVl yawVc yawVl lec rec yc track
    
      qV = gX[0]
      qW = gX[1]
      qB = gX[2]
      
      encVc = gX[3]
      encVl = gX[4]
      yawVc = gX[5]
      yawVl = gX[6]
      
      #lec = gX[7]
      #rec = gX[8]
      #yc = gX[9]
      #track = gX[10]

      # Kalman process noise
      q = array((1e-8, 1e-8, 1e-8, qV, qW, qB))
      self.Q = diagflat(q)
      
      # Sensor variances
      self.encV = (encVc, encVl, 0.0) # Constant, "linear", "square"
      self.yawV = (yawVc, yawVl, 0.0) # Constant, "linear", "square"

      # Model constants
      self.lec = 0.00087589
      self.rec = 0.00087196
      self.lmec = self.lec / 15.0
      self.rmec = self.rec / 15.0
      self.yc = 0.0126276
      self.track = 0.561975
      self.dt = 1.0/50.0
      
      self.publish = 0

    self.initCommon()

  def initCommon(self):
    # Kalman state variables
    self.x = zeros((6,1)) # Kalman state [x; y; th; v; w; Vbias]
    self.P = eye(6) # Covariance matrix
    
    self.firstRun = 1
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
    
    if(self.firstRun == 1):
      # Just record encoder values
      self.lme = msg.left_motor_encoder
      self.rme = msg.right_motor_encoder
    else:
      dl = self.lmec*(msg.left_motor_encoder-self.lme)
      dr = self.rmec*(msg.right_motor_encoder-self.rme)
      self.lme = msg.left_motor_encoder
      self.rme = msg.right_motor_encoder
      venc = (dl + dr)/(2.0*self.dt)
      wenc = (dl - dr)/(self.track*self.dt)
      
      self.x[0] = x + venc*self.dt*cos(th)
      self.x[1] = y + venc*self.dt*sin(th)
      self.x[2] = th + wenc*self.dt
      self.x[3] = venc
      self.x[4] = wenc
      
      # Covariance prediction
      self.P = dot(dot(self.A,self.P),self.A.T) + self.Q
    
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
      
      yv = (msg.yaw_rate/1000.0)/self.yc*3.1415/180.0
      
      z = array([dle, dre, yv]).T
      
      self.le = msg.left_wheel_encoder
      self.re = msg.right_wheel_encoder
      
      lev = self.variance_from_measurement(dle, self.encV[0], self.encV[1], self.encV[2])
      rev = self.variance_from_measurement(dre, self.encV[0], self.encV[1], self.encV[2])
      
      self.R = diagflat(array([lev, rev]))
      
      yc = z - self.sensor_prediction()
      y = yc[:,0:2]
      S = dot(dot(self.Henc,self.P),self.Henc.T) + self.R
      K = dot(dot(self.P,self.Henc.T),linalg.pinv(S))
      self.x = self.x + dot(K,y.T)
      self.P = dot(eye(6)-dot(K,self.Henc),self.P)
      
      # For Yaw
      
      yawVar = self.variance_from_measurement(yv, self.yawV[0], self.yawV[1], self.yawV[2])
      self.R = diagflat(array([yawVar]))
      
      yc = z - self.sensor_prediction()
      #print "%s vs %s" % (z, self.sensor_prediction())
      y = yc[0,2]
      S = dot(dot(self.Hyaw,self.P),self.Hyaw.T) + self.R
      K = dot(dot(self.P,self.Hyaw.T),linalg.pinv(S))
      self.x = self.x + dot(K,y.T)
      self.P = dot(eye(6)-dot(K,self.Hyaw),self.P)
    
  def sensor_prediction(self):
    v = self.x[3]
    w = self.x[4]
    b = self.x[5]
    y = w + b
    le = 1.0/2.0*self.dt*(2.0*v+self.track*w)
    re = 1.0/2.0*self.dt*(2.0*v-self.track*w)
    z = array([le, re, y])
    return z.T
    
  @staticmethod
  def variance_from_measurement(meas, c0, c1, c2):
    return c0 + c1 * meas * meas + c2 * meas * meas * meas * meas
    
  def sensorJacobian(self):
    dt = self.dt
    self.Henc = zeros((2,6))
    self.Henc[0,3] = dt
    self.Henc[1,3] = dt
    self.Henc[0,4] = dt/2.0
    self.Henc[1,4] = -dt/2.0
    self.Hyaw = zeros((1,6))
    self.Hyaw[0,4] = 1.0
    self.Hyaw[0,5] = 1.0
    
  def getState(self):
    return self.x
    
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
	'''print self.trans
	print trans2
	print self.rot
	print rot2
	print tf.transformations.euler_from_quaternion(rot2)[2]'''
	self.x[2] -= tf.transformations.euler_from_quaternion(rot2)[2]
	self.x[0] += trans2[0]
	self.x[1] -= trans2[1]
      except (tf.LookupException, tf.ConnectivityException):
	a = 0
    else: # Copy the transform from amcl to odoms
      (self.trans,self.rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
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