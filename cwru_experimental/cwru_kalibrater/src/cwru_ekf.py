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
    self.x = zeros((6,1)) # Kalman state [x; y; th; v; w; Vbias]
    self.P = eye(6) # Covariance matrix

    # Model constants
    self.lec = 0.00087589
    self.rec = 0.00087196
    self.yc = 0.0126
    self.track = 0.561975
    self.dt = 1.0/50.0
    
    # Kalman model Jacobian
    self.A = zeros((6,6))
    
    # Kalman sensor Jacobian
    self.H = zeros((2,6))
    
    # Kalman process noise
    q = array((1e-6, 1e-6, 1e-6, 10, 10, 1e-10))
    self.Q = diagflat(q)
    
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
    
    # Covariance prediction
    self.P = dot(dot(self.A,self.P),self.A.T) + self.Q
    
  def modelJacobian(self):
    th = self.x[2]
    v = self.x[3]
    dt = self.dt
    
    self.A = eye(6)
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
      
      dle = self.lec*(msg.left_wheel_encoder-self.le)
      dre = self.rec*(msg.right_wheel_encoder-self.re)
      yv = (msg.yaw_rate/1000.0-self.x[5])/self.yc*3.1415/180.0
      
      z = array([dle, dre, yv]).T
      
      self.le = msg.left_wheel_encoder
      self.re = msg.right_wheel_encoder
      
      lev = self.variance_from_measurement(dle, 1e-12, 0.0002, 0.0)
      rev = self.variance_from_measurement(dre, 1e-12, 0.0002, 0.0)
      yawVar = self.variance_from_measurement(yv, .00000019, .000048345, 0.0)
      
      self.R = diagflat(array([lev, rev, yawVar]))
      
      y = z - self.sensor_prediction()
      S = dot(dot(self.H,self.P),self.H.T) + self.R
      K = dot(dot(self.P,self.H.T),linalg.pinv(S))
      self.x = self.x + dot(K,y.T)
      self.P = dot(eye(6)-dot(K,self.H),self.P)
    
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
    dt = self.dt
    self.H = zeros((3,6))
    self.H[0,3] = dt
    self.H[1,3] = dt
    self.H[0,4] = dt/2.0
    self.H[1,4] = -dt/2.0
    self.H[2,4] = 1.0
    self.H[2,5] = 1.0
    
  def publish_filter(self):
    x = self.x[0]
    y = -self.x[1] # Change to ROS
    th = -self.x[2] # Change to ROS
    v = self.x[3]
    w = -self.x[4] # Change to ROS
    
    if(self.trans == 0):
      try:
	(self.trans,self.rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	#(self.trans2,self.rot2) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	#self.trans = self.trans + self.trans2
	self.tf_br.sendTransform(translation = self.trans, 
		  rotation = self.rot,
		  time = rospy.Time.now(),
		  child = 'odom_ekf',
		  parent = 'map')
      except (tf.LookupException, tf.ConnectivityException):
	a = 0
    else:
      self.tf_br.sendTransform(translation = self.trans, 
		rotation = self.rot,
		time = rospy.Time.now(),
		child = 'odom_ekf',
		parent = 'map')
    
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
	  myEKF = EKF()

	except rospy.ROSInterruptException: pass