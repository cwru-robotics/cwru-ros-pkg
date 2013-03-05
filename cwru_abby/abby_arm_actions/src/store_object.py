#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy
import actionlib
from arm_navigation_msgs.msg import *
from abby_gripper.srv import *

class StoreObject:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
        
        rospy.logdebug("Setting up goal request message.")
        self.goal = MoveArmGoal()
        self.goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
        
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "irb_120"
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        
        pos_constraint = PositionConstraint()
        #pos_constraint.header.frame_id = "/irb_120_base_link"
        pos_constraint.header.frame_id = "/frame1"
        pos_constraint.link_name = "gripper_body"
        pos_constraint.position.x = -0.198
        pos_constraint.position.y = -0.758
        pos_constraint.position.z =  0.668
        pos_constraint.constraint_region_shape.type = Shape.BOX
        pos_constraint.constraint_region_shape.dimensions = [0.05, 0.05, 0.05]
        pos_constraint.constraint_region_orientation.x = 0;
        pos_constraint.constraint_region_orientation.y = 0;
        pos_constraint.constraint_region_orientation.z = 0;
        pos_constraint.constraint_region_orientation.w = 1.0;
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        
        o_constraint = OrientationConstraint()
        o_constraint.header = pos_constraint.header
        o_constraint.link_name = pos_constraint.link_name
        o_constraint.orientation.x = -0.708
        o_constraint.orientation.y =  0.057
        o_constraint.orientation.z = -0.386
        o_constraint.orientation.w =  0.589
        #o_constraint.orientation.x =  0.694
        #o_constraint.orientation.y =  0.570
        #o_constraint.orientation.z = -0.275
        #o_constraint.orientation.w =  0.343
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        
        self.goal.motion_plan_request = motion_plan_request
        
        rospy.loginfo("Waiting for arm action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to arm action server.")
        rospy.loginfo("Waiting for gripper service server...")
        rospy.wait_for_service('abby_gripper/gripper')
        self.gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        rospy.loginfo("Connected to gripper service server.")
        
    def sendOnce(self, timeOut = 60):
        rospy.loginfo('Sending bin position goal...')
        self.client.send_goal(self.goal)
        if self.client.wait_for_result(rospy.Duration(timeOut, 0)):
            return self.client.get_result()
        else:
            rospy.logwarn('Timed out attempting to move to bin.')
            return False
    
    def sendUntilSuccess(self, timeOut = 60):
        result = False
        r = rospy.Rate(1)
        while not result:
            result = self.sendOnce(timeOut)
            if not result:
                r.sleep()
        status = result.error_code
        if status.val == status.SUCCESS:
            rospy.loginfo("Arm successfully moved to bin.")
        else:
            rospy.logerr("Arm failed to go to bin.")
        return result
    
    def storeObject(self, timeOut = 60):
        result = self.sendUntilSuccess(timeOut)
        if result.error_code.val == result.error_code.SUCCESS:
            try:
                response = self.gripperClient(gripperRequest.OPEN)
                rospy.loginfo("Stored object")
                return True
            except rospy.ServiceException, e:
                rospy.logerr("Gripper service did not process request")
        rospy.logerr("Failed to store object. Error code: %d", result.error_code.val)
        return False

if __name__ == '__main__':
    rospy.init_node('store_object')
    rospy.loginfo("Node initialized.")
    storeObject = StoreObject()
    storeObject.storeObject()
