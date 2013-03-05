#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package draw_functions
#helper functions for drawing things in rviz

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf.transformations
import scipy
import math
import time
import pdb
from convert_functions import *

##class to draw stuff to rviz
class DrawFunctions():

    def __init__(self, topic, latch = False):
        self.marker_pub = rospy.Publisher(topic, Marker, latch = latch)

    ##clear the current set of points
    def clear_rviz_points(self, ns = 'points', id = 0):

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'points'
        marker.type = Marker.POINTS
        marker.action = Marker.DELETE
        marker.id = id
        self.marker_pub.publish(marker)


    ##fill in a Marker message
    def create_marker(self, type, dims, frame, ns, id, duration = 60., color = [1,0,0], opaque = 0.5, pos = [0.,0.,0.], quat = [0.,0.,0.,1.], frame_locked = False):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = type
        marker.action = Marker.ADD
        marker.scale.x = dims[0]
        marker.scale.y = dims[1]
        marker.scale.z = dims[2]
        marker.color.a = opaque
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.lifetime = rospy.Duration(duration)
        marker.id = id
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]       
        marker.frame_locked = frame_locked
        return marker
 

    ##draw a set of points (3xn or 4xn scipy matrix) in rviz
    def draw_rviz_points(self, points, frame = 'narrow_stereo_optical_frame', size = .005, ns = 'points', id = 0, duration = 20., color = [0,0,1], opaque = 1.0, pose_mat = None, frame_locked = False):
        if pose_mat != None:
            (pos, quat) = mat_to_pos_and_quat(pose_mat)
            marker = self.create_marker(Marker.POINTS, [size, size, size], frame, ns, id, duration, color, opaque, pos, quat, frame_locked = frame_locked)
        else:
            marker = self.create_marker(Marker.POINTS, [size, size, size], frame, ns, id, duration, color, opaque, frame_locked = frame_locked)

        for point_ind in range(scipy.shape(points)[1]):
            new_point = Point()
            new_point.x = points[0, point_ind]
            new_point.y = points[1, point_ind]
            new_point.z = points[2, point_ind]
            marker.points.append(new_point)

        self.marker_pub.publish(marker)


    ##draw a set of axes in rviz with arrows of varying lengths
    #pose is a 4x4 scipy matrix
    def draw_rviz_axes(self, pose_mat, frame, lengths = [.05, .01, .01], ns = 'axes', id = 0, duration = 300., frame_locked = False):
        
        marker = self.create_marker(Marker.ARROW, [.01, .02, 0], frame, ns, id, duration, frame_locked = frame_locked)
        marker.color.a = 1.0

        #find the arrow endpoints
        start = pose_mat[0:3, 3]
        x_end = (pose_mat[:,0][0:3]*lengths[0] + start).T.tolist()[0]    
        y_end = (pose_mat[:,1][0:3]*lengths[1] + start).T.tolist()[0]
        z_end = (pose_mat[:,2][0:3]*lengths[2] + start).T.tolist()[0]
        start = start.T.tolist()[0]

        #draw the arrows (x=red, y=green, z=blue)
        marker.id = id
        marker.points = [Point(*start), Point(*x_end)]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+1
        marker.points = [Point(*start), Point(*y_end)]
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        marker.id = id+2
        marker.points = [Point(*start), Point(*z_end)]
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)


    ##draw a sphere in rviz at pose_mat (4x4 scipy matrix) with radius r
    def draw_rviz_sphere(self, pose_mat, r, frame = 'object_frame', ns = 'spheres', id = 0, duration = 60., color = [1,0,0], opaque = 0.5, frame_locked = False):
        
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.SPHERE, [r*2., r*2., r*2.], frame, ns, id, duration, color, opaque, pos, quat, frame_locked = frame_locked)
        self.marker_pub.publish(marker)


    ##draw a box in rviz at pose_mat (4x4 scipy matrix) defined by either:
    #    2-lists (min, max) of 3-lists (x,y,z) of corner coords
    #    or a 3-list of dimensions (x,y,z)
    #in frame_id frame (defaults to the object frame), id number id, and RGB color
    def draw_rviz_box(self, pose_mat, ranges, frame = 'object_frame', ns = 'boxes', id = 0, duration = 60., color = [1,0,0], opaque = 0.5, frame_locked = False):
        if len(ranges) == 2:
            dims = [upper-lower for (upper, lower) in list(zip(ranges[0], ranges[1]))]
            center = [(upper-lower)/2+lower for (upper, lower) in list(zip(ranges[0], ranges[1]))]

        elif len(ranges) == 3:
            dims = ranges
            center = [0., 0., 0.]

        #rotate the box center to frame
        center = scipy.matrix(center + [1])
        transformed_center = pose_mat * center.T
                                    
        quat = tf.transformations.quaternion_from_matrix(pose_mat)
            
        marker = self.create_marker(Marker.CUBE, dims, frame, ns, id, duration, color, opaque, transformed_center[0:3, 0], quat, frame_locked = frame_locked)
        self.marker_pub.publish(marker)


    ##draw a cylinder in rviz at pose_mat (4x4 scipy matrix, z-axis is cylinder axis) with radius r and length l
    def draw_rviz_cylinder(self, pose_mat, r, l, frame = 'object_frame', ns = 'cylinders', id = 0, duration = 60., color = [1,0,0], opaque = 0.5, frame_locked = False):
        
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.CYLINDER, [r*2., r*2., l], frame, ns, id, duration, color, opaque, pos, quat, frame_locked = frame_locked)
        self.marker_pub.publish(marker)


    ##draw a line strip in rviz, with origin at pose_mat (4x4 scipy matrix) and points (3xn or 4xn scipy matrix) relative to that origin
    def draw_rviz_line_strip(self, pose_mat, points, frame, size = .005, ns = 'line strip', id = 0, duration = 20., color = [0,1,1], opaque = 1.0, frame_locked = False):
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        marker = self.create_marker(Marker.LINE_STRIP, [size, 0, 0], frame, ns, id, duration, color, opaque, pos, quat, frame_locked = frame_locked)
        for point_ind in range(scipy.shape(points)[1]):
            new_point = Point()
            new_point.x = points[0, point_ind]
            new_point.y = points[1, point_ind]
            new_point.z = points[2, point_ind]
            marker.points.append(new_point)

        self.marker_pub.publish(marker)


    ##draw a circle in rviz (pose_mat z-axis is normal to the circle plane) with radius r, thickness size, and number of facets num_facets
    def draw_rviz_circle(self, pose_mat, r, frame, num_facets = 20, size = .005, ns = 'circle', id = 0, duration = 20., color = [0,1,1], opaque = 1.0, frame_locked = False):
        angle_step = math.pi*2./num_facets
        points = scipy.matrix(scipy.zeros([3, num_facets+1]))
        for (ind, angle) in enumerate(scipy.arange(0, math.pi*2.+angle_step, angle_step)):
            points[0,ind] = math.cos(angle)*r
            points[1,ind] = math.sin(angle)*r
        self.draw_rviz_line_strip(pose_mat, points, frame, size, ns, id, duration, color, opaque, frame_locked)


    ##clear all the currently drawn grasps by redrawing them tiny and short-lived
    def clear_grasps(self, ns = 'grasps', num = 150, frame = '/base_link'):

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.ARROW
        marker.action = Marker.DELETE
        for i in range(num):
            marker.id = i
            self.marker_pub.publish(marker)


    ##draw a set of grasps (wrist Poses) as x and y-axis arrows in rviz, 
    #with the x-axis long compared to y
    def draw_grasps(self, grasps, frame, ns = 'grasps', pause = 0, frame_locked = False):

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = frame_locked

        for (grasp_num, grasp) in enumerate(grasps):
            if grasp_num == 0:
                marker.scale.x = 0.015
                marker.scale.y = 0.025
                length_fact = 1.5

            else:
                marker.scale.x = 0.01
                marker.scale.y = 0.015
                length_fact = 1.0

            orientation = grasp.orientation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            mat = tf.transformations.quaternion_matrix(quat)
            start = [grasp.position.x, grasp.position.y, grasp.position.z]
            x_end = list(mat[:,0][0:3]*.05*length_fact + scipy.array(start))    
            y_end = list(mat[:,1][0:3]*.02*length_fact + scipy.array(start))
            marker.id = grasp_num*3
            marker.points = [Point(*start), Point(*x_end)]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)
            marker.id = grasp_num*3+1
            marker.points = [Point(*start), Point(*y_end)]
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)
            marker.id = grasp_num*3+2
            if pause:
                print "press enter to continue"
                raw_input()
        time.sleep(.5)


#test script
if __name__ == "__main__":
    rospy.init_node('draw_functions', anonymous=True)

    pose_mat = scipy.matrix(scipy.identity(4))
    pose_mat[2, 3] = -1.0

    draw_functions = DrawFunctions('visualization_marker')

    while(not rospy.is_shutdown()):

        points = scipy.matrix([[.8, 0., -1.0],
                               [.9, 0., -1.0],
                               [1.0, 0., -1.0],
                               [.9, .1, -1.0]]).T
        draw_functions.draw_rviz_points(points, frame = 'base_footprint')

        pose_mat[0, 3] = .3
        draw_functions.draw_rviz_box(pose_mat, [.1, .2, .3], frame = 'base_footprint')
        
        pose_mat[0, 3] = 0
        draw_functions.draw_rviz_cylinder(pose_mat, .1, .3, frame = 'base_footprint')
        
        pose_mat[0, 3] = -.3
        draw_functions.draw_rviz_sphere(pose_mat, .1, frame = 'base_footprint')

        pose_mat[0, 3] = .6
        pose = mat_to_pose(pose_mat)
        draw_functions.draw_grasps([pose], frame = 'base_footprint')
        
        print "drew shapes, press enter to re-draw"

        raw_input()
