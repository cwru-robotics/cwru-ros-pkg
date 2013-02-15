#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Edward Vneator, Case Western Reserve University
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
#  * Neither the name of Case Western Reserve University nor the names 
#    of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
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

__author__ = "esv@case.edu (Edward Venator)"

import roslib; roslib.load_manifest("abby_object_manipulator")
import rospy
import actionlib
from tabletop_object_detector.srv import TabletopSegmentation
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.msg import DatabaseModelPoseList
from object_manipulation_msgs.msg import *
from geometry_msgs.msg import *
'''This node coordinates messages and services for the object manipulation pipeline
on Abby. It serves a similar purpose to tabletop_complete, but does not perform
object detection.'''

class ObjectManipulationController:
    '''Not threadsafe'''
    def __init__(self):
        rospy.loginfo('Waiting for tabletop_segmentation service')
        rospy.wait_for_service('tabletop_segmentation')
        self.segmentationService = rospy.ServiceProxy('tabletop_segmentation', TabletopSegmentation)
        rospy.loginfo('Connected to tabletop segmentation service')
        rospy.loginfo('Waiting for tabletop_collision_map_processing service')
        rospy.wait_for_service('tabletop_collision_map_processing/tabletop_collision_map_processing')
        self.collisionMapService = rospy.ServiceProxy('tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)
        rospy.loginfo('Connected to collision map processing service')
        self._pickupClient = actionlib.SimpleActionClient('object_manipulation_pickup', PickupAction)
        self._placeClient = actionlib.SimpleActionClient('object_manipulation_place', PlaceAction)
    
    def segmentationReplyToDetectionReply(self, segmentationResult):
        '''Converts a TableTopSegmentationReply into a TabletopDetectionResult'''
        detectionResult = TabletopDetectionResult()
        detectionResult.table = segmentationResult.table
        detectionResult.clusters = segmentationResult.clusters
        detectionResult.models.append(DatabaseModelPoseList())
        for i in range(len(detectionResult.clusters)):
            detectionResult.cluster_model_indices.append(0)
        detectionResult.result = segmentationResult.result
        return detectionResult
    
    def runSegmentation(self):
        '''Calls the segmentation service and sends the result on to the collision map processing server.'''
        segmentationResult = self.segmentationService()
        detectionResult = self.segmentationReplyToDetectionReply(segmentationResult)
        self.mapResponse = self.collisionMapService(detectionResult, False, False, '/base_link')
        return segmentationResult
    
    def getMapResponse(self):
        '''Returns graspable objects from the collision map server. Run runSegmenation() first.'''
        return self.mapResponse
    
    def pickup(self, mapResponse, index):
        '''Sends a command to pick up the graspable object at the given index in mapResponse'''
        goal = PickupGoal()
        goal.arm_name = 'irb_120'
        goal.target = mapResponse.graspable_objects[index]
        goal.collision_object_name = mapResponse.collision_object_names[index]
        goal.collision_support_surface_name = mapResponse.collision_support_surface_name
        goal.allow_gripper_support_collision = False
        goal.use_reactive_execution = False
        goal.use_reactive_lift = False
        goal.ignore_collisions = False
        
        #Set this to false to enable actual execution of grasping
        goal.only_perform_feasibility_test = True
        
        self._pickupClient.wait_for_server()
        self._pickupClient.send_goal(goal)
        if self._pickupClient.wait_for_result(rospy.Duration.from_sec(10.0)):
            result = self._pickupClient.get_result()
            if result.status.status == result.status.SUCCEEEDED:
                self.currentlyHeldObject = goal.target
                return True;
        return False;

    def storeObject(self):
        '''Sends a command to store the currently held object in the bin'''
        rospy.loginfo('Storing the currently held object in the bin')
        goal = PlaceGoal()
        goal.arm_name = 'irb_120'
        bin_location = PoseStamped()
        bin_location.header.frame_id = '/irb_120_base_link'
        bin_location.pose.position.x = 0.0816875
        bin_location.pose.position.y = 0.207909
        bin_location.pose.position.z = 0.243964
        bin_location.pose.orientation.x =  0.847531
        bin_location.pose.orientation.y =  0.297557
        bin_location.pose.orientation.z = -0.140450
        bin_location.pose.orientation.w =  0.416442
        goal.place_locations.append(bin_location)
        goal.collision_object_name = self.currentlyHeldObject.collision_name
        rospy.loginfo('Sent place goal to box manipulator')
        self._placeClient.send_goal_and_wait(goal, rospy.Duration.from_sec(60.0), rospy.Duration.from_sec(60.0))
if __name__ == '__main__':
    rospy.init_node('object_manipulation_controller')
    rospy.loginfo('Started the object manipulation controller')
    controller = ObjectManipulationController()
    timer = rospy.Rate(.2)
    while not rospy.is_shutdown():
        resp = controller.runSegmentation()
        mapResponse = controller.getMapResponse()
        if resp.result == resp.SUCCESS:
            rospy.loginfo("Tabletop detection service returned %d clusters", len(resp.clusters))
            break
        elif resp.result == resp.NO_TABLE:
            rospy.loginfo("No table detected")
        elif resp.result == resp.NO_CLOUD_RECEIVED:
            rospy.logwarn("No cloud received")
        elif resp.result == resp.OTHER_ERROR:
            rospy.logerr("Tabletop segmentation error")
    for index in range(len(mapResponse.graspable_objects)):
        rospy.loginfo("Picking up object number %d", index)
        if controller.pickup(mapResponse, index):
            controller.storeObject()
       #timer.sleep()
        
