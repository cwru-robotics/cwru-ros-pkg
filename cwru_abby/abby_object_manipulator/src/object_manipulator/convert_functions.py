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

## @package convert_functions
#helper functions for creating, transforming, and converting among messages, 
#scipy matrices, and lists

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy
from geometry_msgs.msg import Transform, Pose, PoseStamped, Point, Point32, PointStamped, Vector3, Vector3Stamped, Quaternion, QuaternionStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
import tf.transformations
import tf
import numpy as np
import scipy
from sensor_msgs.msg import PointCloud, PointCloud2
from object_manipulator.point_cloud import *

##convert a PointCloud or PointCloud2 to a 4xn scipy matrix (x y z 1)
def point_cloud_to_mat(point_cloud):
    if type(point_cloud) == type(PointCloud()):
        points = [[p.x, p.y, p.z, 1] for p in point_cloud.points]
    elif type(point_cloud) == type(PointCloud2()):
        points = [[p[0], p[1], p[2], 1] for p in read_points(point_cloud, field_names = 'xyz', skip_nans=True)]
    else:
        print "type not recognized:", type(point_cloud)
        return None
    points = scipy.matrix(points).T
    return points


##convert a 4xn scipy matrix (x y z 1) to a PointCloud 
def mat_to_point_cloud(mat, frame_id):
    pc = PointCloud()
    pc.header.frame_id = frame_id
    for n in range(mat.shape[1]):
        column = mat[:,n]
        point = Point32()
        point.x, point.y, point.z = column[0,0], column[1,0], column[2,0]
        pc.points.append(point)
    return pc 


##transform a PointCloud or PointCloud2 to be a 4xn scipy matrix (x y z 1) in a new frame
def transform_point_cloud(tf_listener, point_cloud, frame):        
    points = point_cloud_to_mat(point_cloud)
    transform = get_transform(tf_listener, point_cloud.header.frame_id, frame)
    if transform == None:
        return (None, None)
    points = transform * points
    return (points, transform)


##get the 4x4 transformation matrix from frame1 to frame2 from TF
def get_transform(tf_listener, frame1, frame2):
    temp_header = Header()
    temp_header.frame_id = frame1
    temp_header.stamp = rospy.Time(0)
    try:
        tf_listener.waitForTransform(frame1, frame2, rospy.Time(0), rospy.Duration(5))
    except tf.Exception, e:
        rethrow_tf_exception(e, "tf transform was not there between %s and %s"%(frame1, frame2))
    frame1_to_frame2 = tf_listener.asMatrix(frame2, temp_header)
    return scipy.matrix(frame1_to_frame2)


def pose_to_mat(pose):
    '''Convert a pose message to a 4x4 numpy matrix.

    Args:
        pose (geometry_msgs.msg.Pose): Pose rospy message class.

    Returns:
        mat (numpy.matrix): 4x4 numpy matrix
    '''
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    pos = np.matrix([pose.position.x, pose.position.y, pose.position.z]).T
    mat = np.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat


def mat_to_pose(mat, transform = None):
    '''Convert a homogeneous matrix to a Pose message, optionally premultiply by a transform.

    Args:
        mat (numpy.ndarray): 4x4 array (or matrix) representing a homogenous transform.
        transform (numpy.ndarray): Optional 4x4 array representing additional transform

    Returns:
        pose (geometry_msgs.msg.Pose): Pose message representing transform.
    '''
    if transform != None:
        mat = np.dot(transform, mat)
    pose = Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose

def transform_to_mat(transform):
    '''Convert a tf transform to a 4x4 scipy mat.

    Args:
        transform (geometry_msgs.msg.Transform): Transform rospy message class.

    Returns:
        mat (numpy.matrix): 4x4 numpy matrix
    '''
    quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    pos = scipy.matrix([transform.translation.x, transform.translation.y, transform.translation.z]).T
    mat = scipy.matrix(tf.transformations.quaternion_matrix(quat))
    mat[0:3, 3] = pos
    return mat

def mat_to_transform(mat, transform = None):
    '''Convert a 4x5 scipy matrix to a transform message.

    Args:
        mat (numpy.matrix): 4x4 numpy matrix
        transform (numpy.ndarray): Optional 4x4 array representing additional transform

    Returns:
        transform (geometry_msgs.msg.Transform): Transform rospy message class.
    '''
    if transform != None:
        mat = transform * mat
    trans = np.array(mat[:3,3] / mat[3,3]).flat
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose = Transform(Vector3(*trans), Quaternion(*quat))
    return pose

##convert a 4x4 scipy matrix to position and quaternion lists
def mat_to_pos_and_quat(mat):
    quat = tf.transformations.quaternion_from_matrix(mat).tolist()
    pos = mat[0:3,3].T.tolist()[0]
    return (pos, quat)


##stamp a message by giving it a header with a timestamp of now
def stamp_msg(msg, frame_id):
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()


##make a PoseStamped out of a Pose
def stamp_pose(pose, frame_id):
    pose_stamped = PoseStamped()
    stamp_msg(pose_stamped, frame_id)
    pose_stamped.pose = pose
    return pose_stamped


##make a Vector3Stamped out of a Vector3
def stamp_vector3(vector3, frame_id):
    vector3_stamped = Vector3Stamped()
    stamp_msg(vector3_stamped, frame_id)
    vector3_stamped.vector = vector3
    return vector3_stamped


##set x, y, and z fields with a list
def set_xyz(msg, xyz):
    msg.x = xyz[0]
    msg.y = xyz[1]
    msg.z = xyz[2]


##set x, y, z, and w fields with a list
def set_xyzw(msg, xyzw):
    set_xyz(msg, xyzw)
    msg.w = xyzw[3]


##get x, y, and z fields in the form of a list
def get_xyz(msg):
    return [msg.x, msg.y, msg.z]


##get x, y, z, and w fields in the form of a list
def get_xyzw(msg):
    return [msg.x, msg.y, msg.z, msg.w]


##transform a poseStamped by a 4x4 scipy matrix 
def transform_pose_stamped(pose_stamped, transform, pre_or_post = "post"):

    #convert the Pose to a 4x3 scipy matrix
    pose_mat = pose_to_mat(pose_stamped.pose)
    
    #post-multiply by the transform
    if pre_or_post == "post":
        transformed_mat = pose_mat * transform
    else:
        transformed_mat = transform * pose_mat
    (pos, quat) = mat_to_pos_and_quat(transformed_mat)

    #create a new poseStamped
    new_pose_stamped = PoseStamped()
    new_pose_stamped.header = pose_stamped.header
    new_pose_stamped.pose = Pose(Point(*pos), Quaternion(*quat))

    return new_pose_stamped


##convert a pointStamped to a pos list in a desired frame
def point_stamped_to_list(tf_listener, point, frame):

    #convert the pointStamped to the desired frame, if necessary
    if point.header.frame_id != frame:
        try:
            tf_listener.waitForTransform(frame, point.header.frame_id, \
                                     rospy.Time(0), rospy.Duration(5))
            trans_point = tf_listener.transformPoint(frame, point)
        except tf.Exception, e:
            rethrow_tf_exception(e, "point_stamped_to_list: error in transforming point from " + point.header.frame_id + " to " + frame + " error msg: %s"%e)
    else:
        trans_point = point

    #extract position as a list
    pos = [trans_point.point.x, trans_point.point.y, trans_point.point.z]

    return pos


##convert a Vector3Stamped to a rot list in a desired frame
def vector3_stamped_to_list(tf_listener, vector3, frame):

    #convert the vector3Stamped to the desired frame, if necessary
    if vector3.header.frame_id != frame:
        try:
            tf_listener.waitForTransform(frame, vector3.header.frame_id, \
                        rospy.Time(0), rospy.Duration(5))
            trans_vector3 = tf_listener.transformVector3(frame, vector3)
        except tf.Exception, e:
            rethrow_tf_exception(e, "vector3_stamped_to_list: error in transforming point from " + vector3.header.frame_id + " to " + frame + " error msg: %s"%e)
    else:
        trans_vector3 = vector3

    #extract vect as a list
    vect = [trans_vector3.vector.x, trans_vector3.vector.y, trans_vector3.vector.z]

    return vect


##convert a QuaternionStamped to a quat list in a desired frame
def quaternion_stamped_to_list(tf_listener, quaternion, frame):

    #convert the QuaternionStamped to the desired frame, if necessary
    if quaternion.header.frame_id != frame:
        try:
            tf_listener.waitForTransform(frame, quaternion.header.frame_id, \
                        rospy.Time(0), rospy.Duration(5))
            trans_quat = tf_listener.transformQuaternion(frame, quaternion)
        except tf.Exception, e:
            rethrow_tf_exception(e, "quaternion_stamped_to_list: error in transforming point from " + quaternion.header.frame_id + " to " + frame + " error msg: %s"%e)
    else:
        trans_quat = quaternion

    #extract quat as a list
    quat = [trans_quat.quaternion.x, trans_quat.quaternion.y, trans_quat.quaternion.z, trans_quat.quaternion.w]

    return quat


##change the frame of a Vector3Stamped 
def change_vector3_stamped_frame(tf_listener, vector3_stamped, frame):

    if vector3_stamped.header.frame_id != frame:
        vector3_stamped.header.stamp = rospy.Time(0)
        try:
            tf_listener.waitForTransform(frame, vector3_stamped.header.frame_id,\
                    vector3_stamped.header.stamp, rospy.Duration(5))
            trans_vector3 = tf_listener.transformVector3(frame, vector3_stamped)
        except tf.Exception, e:
            rethrow_tf_exception(e, "change_vector3_stamped: error in transforming vector3 from " + vector3_stamped.header.frame_id + " to " + frame + " error msg: %s"%e)
    else:
        trans_vector3 = vector3_stamped

    return trans_vector3


##change the frame of a PoseStamped
def change_pose_stamped_frame(tf_listener, pose, frame):

    #convert the PoseStamped to the desired frame, if necessary
    if pose.header.frame_id != frame:
        pose.header.stamp = rospy.Time(0)
        try:
            tf_listener.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(5))
            trans_pose = tf_listener.transformPose(frame, pose)
        except tf.Exception, e:
            rethrow_tf_exception(e, "change_pose_stamped_frame: error in transforming pose from " + pose.header.frame_id + " to " + frame + " error msg: %s"%e)
    else:
        trans_pose = pose

    return trans_pose


##convert a PoseStamped to pos and rot (quaternion) lists in a desired frame
def pose_stamped_to_lists(tf_listener, pose, frame):

    #change the frame, if necessary
    trans_pose = change_pose_stamped_frame(tf_listener, pose, frame)

    #extract position and orientation as quaternion
    pos = [trans_pose.pose.position.x, trans_pose.pose.position.y, trans_pose.pose.position.z]
    rot = [trans_pose.pose.orientation.x, trans_pose.pose.orientation.y, \
               trans_pose.pose.orientation.z, trans_pose.pose.orientation.w]

    return (pos, rot)


##convert pos and rot lists (relative to in_frame) to a PoseStamped 
#(relative to to_frame)
def lists_to_pose_stamped(tf_listener, pos, rot, in_frame, to_frame):

    #stick lists in a PoseStamped
    m = PoseStamped()
    m.header.frame_id = in_frame
    m.header.stamp = rospy.get_rostime()
    m.pose = Pose(Point(*pos), Quaternion(*rot))

    try:
        pose_stamped = tf_listener.transformPose(to_frame, m)
    except tf.Exception, e:            
        rethrow_tf_exception(e, "error in transforming pose from " + in_frame + " to " + to_frame + " error msg: %s"%e)
    return pose_stamped


##create a PoseStamped message
def create_pose_stamped(pose, frame_id = 'base_link'):
    m = PoseStamped()
    m.header.frame_id = frame_id
    #m.header.stamp = rospy.get_rostime()
    m.header.stamp = rospy.Time()
    m.pose = Pose(Point(*pose[0:3]), Quaternion(*pose[3:7]))
    return m


##create a PointStamped message
def create_point_stamped(point, frame_id = 'base_link'):
    m = PointStamped()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time()
    #m.header.stamp = rospy.get_rostime()
    m.point = Point(*point)
    return m


##create a Vector3Stamped message
def create_vector3_stamped(vector, frame_id = 'base_link'):
    m = Vector3Stamped()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time()
    #m.header.stamp = rospy.get_rostime()
    m.vector = Vector3(*vector)
    return m


##pretty-print list to string
def pplist(list):
    return ' '.join(['%5.3f'%x for x in list])


##pretty-print numpy matrix to string
def ppmat(mat):
    str = ''
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            str += '%2.3f\t'%mat[i,j]
        str += '\n'
    return str


##catch a tf exception, print a message, rethrowing it first to preserve tracebacks
def rethrow_tf_exception(exception, msg):
    try:
        raise
    finally:
        try:
            rospy.logerr(msg)
        except:
            pass #ignore errors in printing

