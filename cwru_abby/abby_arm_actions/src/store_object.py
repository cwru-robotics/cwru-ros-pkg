#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy
import actionlib
from arm_navigation_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('store_object')
    rospy.loginfo("Node initialized.")
    client = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server.")
    
    goal = MoveArmGoal()
    goal.motion_plan_request.group_name = "irb_120"
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.planner_id = ""
    goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
    
    #constraint = SimplePoseConstraint()
    pos_constraint = PositionConstraint()
    
    pos_constraint.header.frame_id = "/base_link"
    pos_constraint.link_name = "gripper_body"
    
    '''constraint.pose.position.x = 0.0674
    constraint.pose.position.y = -0.220
    constraint.pose.position.z = 0.0742
    constraint.pose.orientation.x = 0.923
    constraint.pose.orientation.y = 0.384
    constraint.pose.orientation.z = 0.001
    constraint.pose.orientation.w = 0.023
    constraint.absolute_position_tolerance.x = 0.05
    constraint.absolute_position_tolerance.y = 0.05
    constraint.absolute_position_tolerance.z = 0.05
    constraint.absolute_roll_tolerance = 0.04
    constraint.absolute_pitch_tolerance = 0.04
    constraint.absolute_yaw_tolerance = 0.04
    '''
    pos_constraint.position.x = 0.0674
    pos_constraint.position.y = -0.220
    pos_constraint.position.z = 0.0742
    pos_constraint.constraint_region_shape.type = Shape.BOX
    pos_constraint.constraint_region_shape.dimensions = [0.05, 0.05, 0.05]
    pos_constraint.constraint_region_orientation.x = 0;
    pos_constraint.constraint_region_orientation.y = 0;
    pos_constraint.constraint_region_orientation.z = 0;
    pos_constraint.constraint_region_orientation.w = 1.0;
    pos_constraint.weight = 1
    
    goal.motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
    
    o_constraint = OrientationConstraint()
    
    o_constraint.header = pos_constraint.header
    o_constraint.link_name = pos_constraint.link_name
    o_constraint.orientation.x = 0.923
    o_constraint.orientation.y = 0.384
    o_constraint.orientation.z = 0.001
    o_constraint.orientation.w = 0.023
    o_constraint.absolute_roll_tolerance = 0.04
    o_constraint.absolute_pitch_tolerance = 0.04
    o_constraint.absolute_yaw_tolerance = 0.04
    o_constraint.weight = 1
    
    goal.motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
    
    #addGoalConstraintToMoveArmGoal(constraint, goal)  
    
    rospy.loginfo("Sending position goal...");
    client.send_goal(goal)
    finishedInTime = client.wait_for_result(rospy.Duration(60,0))
    r= rospy.Rate(0.1);
    while not finishedInTime:
        client.cancel_goal()
        rospy.loginfo("Timed out trying to move arm to bin. Resending goal...")
        client.send_goal(goal)
        finishedInTime = client.wait_for_result(rospy.Duration(60,0))
        r.sleep();
    status = client.get_result().error_code.val
    if status == ArmNavigationErrorCodes.SUCCESS:
        rospy.loginfo("Arm successfully moved to bin.")
    else:
        rospy.logwarn("Arm failed to move to bin.")
