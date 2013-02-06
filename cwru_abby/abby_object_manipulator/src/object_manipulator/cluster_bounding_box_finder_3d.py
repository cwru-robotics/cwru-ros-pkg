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

## @package cluster_bounding_box_finder
#Use PCA to find the principal directions and bounding box for a cluster

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy
import scipy
import pdb
import random
import math
import tf
import scipy.linalg
from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxResponse
from convert_functions import *

## class for using PCA to find the principal directions and bounding
# box for a point cluster
class ClusterBoundingBoxFinder3D:

    def __init__(self, tf_listener = None, tf_broadcaster = None): 

        #init a TF transform listener
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        #init a TF transform broadcaster for the object frame
        if tf_broadcaster == None:
            self.tf_broadcaster = tf.TransformBroadcaster()
        else:
            self.tf_broadcaster = tf_broadcaster


    ##run eigenvector PCA for a 3xn scipy matrix, return the directions (columns in 3x3 matrix)
    def pca(self, points):
        cov_mat = points[0:3,:] * points[0:3,:].T
        (values, vectors) = scipy.linalg.eig(cov_mat)
        vectors[0:3,2] = scipy.cross(vectors[0:3,0], vectors[0:3,1])
        return vectors


    ##shift the points to be around the mean for x and y (matrix is 4xn)
    def mean_shift_xyz(self, points):
        mean = [0]*3
        shifted_points = points.copy()
        for i in range(3):
            mean[i] = scipy.mean(points[i,:])
            shifted_points[i,:] -= mean[i]    
        return (shifted_points, mean)


    ##remove outliers from the cluster (4xn scipy matrix)
    def remove_outliers(self, points):

        points_filtered = points[:]
        empty_space_width = .02
        check_percent = .02
        edge_width = .005

        #remove outliers in each dimension (x,y,z)
        for dim in range(3):

            #sort the points by their values in that dimension
            num_points = scipy.shape(points_filtered)[1]
            points_sorted = points_filtered[:, points_filtered[dim, :].argsort().tolist()[0]]

            #chop off the top points if they are more than empty_space_width away from the bounding box edge
            ind = int(math.floor(num_points*(1-check_percent)))
            if points_sorted[dim, -1] - points_sorted[dim, ind] > empty_space_width:
                
                #find the first point that isn't within edge_width of the point at ind, and lop off all points after
                searcharr = scipy.array(points_sorted[dim, :]).flatten()
                searchval = points_sorted[dim, ind] + edge_width
                thres_ind = scipy.searchsorted(searcharr, searchval)
                if thres_ind != 0 and thres_ind != len(searcharr):
                    points_filtered = points_sorted[:, 0:thres_ind] 
                    rospy.loginfo("chopped points off of dim %d, highest val = %5.3f, searchval = %5.3f"%(dim, points_sorted[dim, -1], searchval))
            else:
                points_filtered = points_sorted

            #do both sides 
            ind = int(math.floor(num_points*check_percent))
            if points_filtered[dim, ind] - points_filtered[dim, 0] > empty_space_width:

                #find the first point that isn't within edge_width of the point at ind, and lop off all points before
                searcharr = scipy.array(points_sorted[dim, :]).flatten()
                searchval = points_sorted[dim, ind] - edge_width
                thres_ind = scipy.searchsorted(searcharr, searchval)
                if thres_ind != 0 and thres_ind != len(searcharr):
                    points_filtered = points_filtered[:, thres_ind:-1]
                    rospy.loginfo("chopped points off of dim -%d, lowest val = %5.3f, searchval = %5.3f"%(dim, points_sorted[dim, 0], searchval))
        return points_filtered


    ##find the object frame and bounding box for a PointCloud or PointCloud2
    #use the local rosparam z_up_frame to specify the desired frame to use where the z-axis is special (box z will be frame z)
    #if not specified, the point cloud's frame is assumed to be the desired z_up_frame
    def find_object_frame_and_bounding_box(self, point_cloud):
        
        #leaving point cloud in the cluster frame
        cluster_frame = point_cloud.header.frame_id
        self.base_frame = cluster_frame
        (points, cluster_to_base_frame) = transform_point_cloud(self.tf_listener, point_cloud, self.base_frame)

        #run PCA on all 3 dimensions 
        (shifted_points, xyz_mean) = self.mean_shift_xyz(points)
        directions = self.pca(shifted_points[0:3, :])

        #convert the points to object frame:
        #rotate all the points to be in the frame of the eigenvectors (should already be centered around xyz_mean)
        rotmat = scipy.matrix(scipy.identity(4))
        rotmat[0:3,0:3] = directions
        object_points = rotmat**-1 * shifted_points

        #remove outliers from the cluster
        #object_points = self.remove_outliers(object_points)

        #find the object bounding box in the new object frame as [[xmin, ymin, zmin], [xmax, ymax, zmax]] (coordinates of opposite corners)
        object_bounding_box = [[0]*3 for i in range(2)]
        object_bounding_box_dims = [0]*3
        for dim in range(3):
            object_bounding_box[0][dim] = object_points[dim,:].min()
            object_bounding_box[1][dim] = object_points[dim,:].max()
            object_bounding_box_dims[dim] = object_bounding_box[1][dim] - object_bounding_box[0][dim]

        #now shift the object frame and bounding box so that the center is the center of the object
        offset_mat = scipy.mat(scipy.identity(4))
        for i in range(3):
            offset = object_bounding_box[1][i] - object_bounding_box_dims[i]/2.  #center
            object_bounding_box[0][i] -= offset   #mins
            object_bounding_box[1][i] -= offset   #maxes
            object_points[i, :] -= offset
            offset_mat[i,3] = offset
        rotmat = rotmat * offset_mat

        #record the transforms from object frame to base frame and to the original cluster frame,
        #broadcast the object frame to tf, and draw the object frame in rviz
        unshift_mean = scipy.identity(4)
        for i in range(3):
            unshift_mean[i,3] = xyz_mean[i]
        object_to_base_frame = unshift_mean*rotmat
        object_to_cluster_frame = cluster_to_base_frame**-1 * object_to_base_frame

        #broadcast the object frame to tf
        (object_frame_pos, object_frame_quat) = mat_to_pos_and_quat(object_to_cluster_frame)
        self.tf_broadcaster.sendTransform(object_frame_pos, object_frame_quat, rospy.Time.now(), "object_frame", cluster_frame) 

        return (object_points, object_bounding_box_dims, object_bounding_box, object_to_base_frame, object_to_cluster_frame)


    
