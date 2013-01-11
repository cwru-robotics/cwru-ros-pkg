#! /usr/bin/env python

import roslib; roslib.load_manifest('abby_arm_actions')
import rospy
import actionlib
from arm_navigation_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('stow_arm')
    rospy.loginfo("Node initialized.")
    client = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server.")
    
    goal = MoveArmGoal()
    goal.motion_plan_request.group_name = "irb-120"
    goal.motion_plan_request.num_planning_attempts = 1
    goal.motion_plan_request.planner_id = ""
    goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
    for i in range(0,6):
    	joint_constraint = JointConstraint()
    	joint_constraint.joint_name = "joint"+str(i+1)
    	goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)
        goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.05
        goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.05
        
    #Joint Positions
    goal.motion_plan_request.goal_constraints.joint_constraints[0].position = 0.00
    goal.motion_plan_request.goal_constraints.joint_constraints[1].position = -1.57
    goal.motion_plan_request.goal_constraints.joint_constraints[2].position = 1.22
    goal.motion_plan_request.goal_constraints.joint_constraints[3].position = 0.0
    goal.motion_plan_request.goal_constraints.joint_constraints[4].position = 0.03
    goal.motion_plan_request.goal_constraints.joint_constraints[5].position = -0.84
    
    rospy.loginfo("Sending stow goal...")
    client.send_goal(goal)
    finishedInTime = client.wait_for_result(rospy.Duration(10,0))
    while not finishedInTime:
        client.cancel_goal()
        rospy.loginfo("Timed out trying to stow arm. Resending stow goal...")
        client.send_goal(goal)
        finishedInTime = client.wait_for_result(rospy.Duration(10,0))
    
    if client.get_state() == actionlib.SimpleGoalState.DONE:
        rospy.loginfo("Arm successfully stowed.")
    else:
        rospy.loginfo("Arm failed to stow.")
