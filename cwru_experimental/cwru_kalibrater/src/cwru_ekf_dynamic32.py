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
  def __init__(self):
    # Kalman state variables
    self.x = zeros((6,1)).astype(float32) # Kalman state [x; y; th; v; w; Vbias]
    self.P = eye(6).astype(float32) # Covariance matrix

    # Model constants
    self.lec = 0.00087589
    self.rec = 0.00087196
    self.yc = 0.0126276
    self.track = 0.561975
    self.dt = 1.0/50.0
    
    # Kalman model Jacobian
    self.A = zeros((6,6)).astype(float32)
    
    # Kalman sensor Jacobian
    self.H = zeros((3,6)).astype(float32)
    
    # Kalman process noise
    q = array((1e-6, 1e-6, 1e-6, 10, 10, 1e-10)).astype(float32)
    self.Q = diagflat(q).astype(float32)
    
    self.firstRun = 1
    self.trans = 0
    
    rospy.init_node('cwru_ekf')
    rospy.Subscriber('crio_sensors', cRIOSensors, self.update_filter)
    self.odom_pub = rospy.Publisher('odom_ekf', Odometry)
    self.tf_br = tf.TransformBroadcaster()
    self.listener = tf.TransformListener()
    rospy.spin()
	      
  def update_filter(self, msg):
    self.modelJacobian()
    self.prediction()
    
    self.sensor_update(msg)
    self.publish_filter()
    
  def prediction(self):
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
    
    self.x = self.x.astype(float32)
    
    # Covariance prediction
    self.P = dot(dot(self.A,self.P).astype(float32),self.A.T).astype(float32) + self.Q
    
  def modelJacobian(self):
    th = self.x[2]
    v = self.x[3]
    dt = self.dt
    
    self.A = eye(6).astype(float32)
    self.A[0,2] = -v*dt*sin(th)
    self.A[0,3] = dt*cos(th)
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
      
      yv = (msg.yaw_rate/1000.0-self.x[5])/self.yc*3.1415/180.0
      
      z = array([dle, dre, yv]).T
      
      self.le = msg.left_wheel_encoder
      self.re = msg.right_wheel_encoder
      
      lev = self.variance_from_measurement(dle, 1e-12, 0.0002, 0.0)
      rev = self.variance_from_measurement(dre, 1e-12, 0.0002, 0.0)
      
      # Effects seen at 1e-12 and .0002, but not with yaw commented out
      # Fixed by increasing the static variance
      # works with .002 D:
      
      self.R = diagflat(array([lev, rev])).astype(float32)
      
      yc = z.astype(float32) - self.sensor_prediction().astype(float32)
      y = yc[:,0:2]
      S = dot(dot(self.Henc,self.P).astype(float32),self.Henc.T).astype(float32) + self.R
      K = dot(dot(self.P,self.Henc.T).astype(float32),linalg.pinv(S).astype(float32)).astype(float32)
      self.x = self.x.astype(float32) + dot(K,y.T).astype(float32)
      self.P = dot(eye(6).astype(float32)-dot(K,self.Henc).astype(float32),self.P).astype(float32)
      
      # For Yaw
      
      yawVar = self.variance_from_measurement(yv, .00000019, .000048345, 0.0)
      self.R = diagflat(array([yawVar])).astype(float32)
      
      yc = z.astype(float32) - self.sensor_prediction().astype(float32)
      y = yc[0,2]
      S = dot(dot(self.Hyaw.astype(float32),self.P.astype(float32)),self.Hyaw.T).astype(float32) + self.R
      K = dot(dot(self.P,self.Hyaw.T).astype(float32),linalg.pinv(S).astype(float32)).astype(float32)
      self.x = self.x.astype(float32) + dot(K,y.T).astype(float32)
      self.P = dot(eye(6).astype(float32)-dot(K,self.Hyaw).astype(float32),self.P).astype(float32)
    
  def sensor_prediction(self):
    v = self.x[3]
    w = self.x[4]
    le = 1.0/2.0*self.dt*(2.0*v+self.track*w)
    re = 1.0/2.0*self.dt*(2.0*v-self.track*w)
    z = array([le, re, w])
    return z.T
    
  @staticmethod
  def variance_from_measurement(meas, c0, c1, c2):
    return c0 + c1 * meas * meas + c2 * meas * meas * meas * meas
    
  def sensorJacobian(self):
    dt = float32(self.dt)
    self.Henc = zeros((2,6)).astype(float32)
    self.Henc[0,3] = dt
    self.Henc[1,3] = dt
    self.Henc[0,4] = dt/2.0
    self.Henc[1,4] = -dt/2.0
    self.Hyaw = zeros((1,6)).astype(float32)
    self.Hyaw[0,4] = 1.0
    self.Hyaw[0,5] = 1.0
    
  def publish_filter(self):
    x = self.x[0]
    y = -self.x[1] # Change to ROS
    th = -self.x[2] # Change to ROS
    v = self.x[3]
    w = -self.x[4] # Change to ROS
    
    if(self.trans == 0):
      try:
	(self.trans,self.rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
	(self.trans2,self.rot2) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	#self.trans = self.trans + self.trans2
	self.tf_br.sendTransform(translation = self.trans, 
		  rotation = self.rot,
		  time = rospy.Time.now(),
		  child = 'odom_ekf_initial',
		  parent = 'map')
	self.tf_br.sendTransform(translation = self.trans, 
		  rotation = self.rot,
		  time = rospy.Time.now(),
		  child = 'odom_ekf',
		  parent = 'odom_ekf_initial')
      except (tf.LookupException, tf.ConnectivityException):
	a = 0
    else: # Copy the transform from amcl to odoms
      (self.trans,self.rot) = self.listener.lookupTransform('/map', '/odom', rospy.Time(0))
      self.tf_br.sendTransform(translation = self.trans, 
		rotation = self.rot,
		time = rospy.Time.now(),
		child = 'odom_ekf_initial',
		parent = 'map')
    
    current_time = rospy.Time.now()
    quaternion = tf.transformations.quaternion_about_axis(0, (0,0,1))
    self.tf_br.sendTransform(translation = (0, 0, 0), 
		    rotation = tuple(quaternion),
		    time = current_time,
		    child = 'odom_ekf',
		    parent = 'odom_ekf_initial')
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
	  myEKF = EKF()

	except rospy.ROSInterruptException: pass