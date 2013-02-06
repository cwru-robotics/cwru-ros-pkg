# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

## @package object_manipulator
# Helper functions for using image regions of PointCloud2s 

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy
import scipy
import math
from sensor_msgs.msg import RegionOfInterest
import pdb


#compute an image mask for the points in the point cluster (PointCloud1 in the camera frame)
def compute_cluster_mask(camera_info, cluster, dist = 1):
    indices = []
    
    #put the corresponding image pixels for each cluster point into the set of indices
    #along with the other pixels up to dist away from it
    for point in cluster.points:
        (x,y) = compute_image_point(camera_info, point)
        for i in range(-dist,dist+1):
            if x+i < 0 or x+i > camera_info.width:
                continue
            for j in range(-dist,dist+1):
                if y+j < 0 or y+j > camera_info.height:
                    continue
                index = image_points_to_indices(camera_info.width, [x+i], [y+j])[0]
                indices.append(index)

    #kill duplicates
    unique_indices = set(indices)
    return list(unique_indices)


#turn a list of x,y coords in the image to indices (index 0 = (0,0), 1 = (1,0), width+1 = (0,1))
def image_points_to_indices(width, xvals, yvals):
    return [y*width + x for (x,y) in zip(xvals, yvals)]


#turn a set of indices into image (x,y) coords
def indices_to_image_points(width, indices):
    xvals = [ind - width*(math.floor(ind / width)) for ind in indices]
    yvals = [math.floor(ind / width) for ind in indices]
    return (xvals, yvals)


#compute the relevant point in the image for the given Point (x=0, y=0 is top left corner)
def compute_image_point(camera_info, point):

    trans_mat = scipy.matrix(scipy.identity(4))
    for i in range(3):
        for j in range(4):
            trans_mat[i,j] = camera_info.P[i*4+j]
    point4 = scipy.matrix(scipy.ones([4,1]))
    point4[0] = point.x
    point4[1] = point.y
    point4[2] = point.z

    image_pt = trans_mat * point4
    x = int(math.floor(image_pt[0,0] / image_pt[2,0]))
    y = int(math.floor(image_pt[1,0] / image_pt[2,0]))

    return (x,y)

    
#generate indices for a rectangular mask
def generate_rect_mask(x_offset, y_offset, height, width, row_step):
        
    point_indices = []
    for y in range(height+1):
        row = range(x_offset+y*row_step, x_offset+y*row_step+width+1) 
        point_indices.extend(row)
    return point_indices


#compute an image rectangle from a set of indices for a PointCloud2
def compute_roi_from_indices(indices, width, height, padding = 30):
        
    xvals = [ind - width*(math.floor(ind / width)) for ind in indices]
    yvals = [math.floor(ind / width) for ind in indices]
    roi = RegionOfInterest()
    roi.x_offset = max(0, min(xvals) - padding)
    roi.y_offset = max(0, min(yvals) - padding)
    
    roi.height = max(yvals) - roi.y_offset + padding
    if roi.height + roi.y_offset > height:
        roi.height = height - roi.y_offset
    roi.width = max(xvals) - roi.x_offset + padding
    if roi.width + roi.x_offset > width:
        roi.width = width - roi.x_offset
    roi.do_rectify = 0

    return roi
