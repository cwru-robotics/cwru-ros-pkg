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
class ClusterBoundingBoxFinder:

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


    ##run eigenvector PCA for a 2xn scipy matrix, return the directions 
    #(list of 2x1 scipy arrays)
    def pca(self, points):
        cov_mat = points[0:3,:] * points[0:3,:].T/scipy.shape(points)[1]
        (values, vectors) = scipy.linalg.eig(cov_mat)
        if values[1] >  values[0]:
            directions = [vectors[:,1], vectors[:,0]] 
        else:
            directions = [vectors[:,0], vectors[:,1]]

        return directions 


    ##shift the points to be around the mean for x and y (matrix is 4xn)
    def mean_shift_xy(self, points):
        mean = [0]*2
        shifted_points = points.copy()
        for i in range(2):
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

            #do both sides for x and y
            if dim != 2:
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
        
        #get the name of the frame to use with z-axis being "up" or "normal to surface" 
        #(the cluster will be transformed to this frame, and the resulting box z will be this frame's z)
        #if param is not set, assumes the point cloud's frame is such
        self.base_frame = rospy.get_param("~z_up_frame", point_cloud.header.frame_id)

        #convert from PointCloud to 4xn scipy matrix in the base_frame
        cluster_frame = point_cloud.header.frame_id

        (points, cluster_to_base_frame) = transform_point_cloud(self.tf_listener, point_cloud, self.base_frame)
        if points == None:
            return (None, None, None)
        #print "cluster_to_base_frame:\n", ppmat(cluster_to_base_frame)

        #find the lowest point in the cluster to use as the 'table height'
        table_height = points[2,:].min()

        #run PCA on the x-y dimensions to find the tabletop orientation of the cluster
        (shifted_points, xy_mean) = self.mean_shift_xy(points)
        directions = self.pca(shifted_points[0:2, :])

        #convert the points to object frame:
        #rotate all the points about z so that the shortest direction is parallel to the y-axis (long side of object is parallel to x-axis) 
        #and translate them so that the table height is z=0 (x and y are already centered around the object mean)
        y_axis = scipy.mat([directions[1][0], directions[1][1], 0.])
        z_axis = scipy.mat([0.,0.,1.])
        x_axis = scipy.cross(y_axis, z_axis)
        rotmat = scipy.matrix(scipy.identity(4))
        rotmat[0:3, 0] = x_axis.T
        rotmat[0:3, 1] = y_axis.T
        rotmat[0:3, 2] = z_axis.T
        rotmat[2, 3] = table_height
        object_points = rotmat**-1 * shifted_points

        #remove outliers from the cluster
        object_points = self.remove_outliers(object_points)

        #find the object bounding box in the new object frame as [[xmin, ymin, zmin], [xmax, ymax, zmax]] (coordinates of opposite corners)
        object_bounding_box = [[0]*3 for i in range(2)]
        object_bounding_box_dims = [0]*3
        for dim in range(3):
            object_bounding_box[0][dim] = object_points[dim,:].min()
            object_bounding_box[1][dim] = object_points[dim,:].max()
            object_bounding_box_dims[dim] = object_bounding_box[1][dim] - object_bounding_box[0][dim]

        #now shift the object frame and bounding box so that the z-axis is centered at the middle of the bounding box
        x_offset = object_bounding_box[1][0] - object_bounding_box_dims[0]/2.
        y_offset = object_bounding_box[1][1] - object_bounding_box_dims[1]/2.
        for i in range(2):
            object_bounding_box[i][0] -= x_offset
            object_bounding_box[i][1] -= y_offset
        object_points[0, :] -= x_offset
        object_points[1, :] -= y_offset
        offset_mat = scipy.mat(scipy.identity(4))
        offset_mat[0,3] = x_offset
        offset_mat[1,3] = y_offset
        rotmat = rotmat * offset_mat
        #pdb.set_trace()

        #record the transforms from object frame to base frame and to the original cluster frame,
        #broadcast the object frame to tf, and draw the object frame in rviz
        unshift_mean = scipy.identity(4)
        unshift_mean[0,3] = xy_mean[0]
        unshift_mean[1,3] = xy_mean[1]
        object_to_base_frame = unshift_mean*rotmat
        object_to_cluster_frame = cluster_to_base_frame**-1 * object_to_base_frame

        #broadcast the object frame to tf
        (object_frame_pos, object_frame_quat) = mat_to_pos_and_quat(object_to_cluster_frame)
        self.tf_broadcaster.sendTransform(object_frame_pos, object_frame_quat, rospy.Time.now(), "object_frame", cluster_frame) 

        return (object_points, object_bounding_box_dims, object_bounding_box, object_to_base_frame, object_to_cluster_frame)


    
