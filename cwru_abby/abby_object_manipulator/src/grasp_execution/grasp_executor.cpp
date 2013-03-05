/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include "object_manipulator/grasp_execution/grasp_executor.h"

#include "object_manipulator/tools/exceptions.h"
#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"

using object_manipulation_msgs::GraspResult;
using arm_navigation_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

/*! Disables collision between gripper and target */
arm_navigation_msgs::OrderedCollisionOperations 
GraspExecutor::collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.object1 = handDescription().gripperCollisionName(pickup_goal.arm_name);
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  if (!pickup_goal.collision_object_name.empty())
  {
    coll.object2 = pickup_goal.collision_object_name;
    ord.collision_operations.push_back(coll);
  }
  if (pickup_goal.allow_gripper_support_collision)
  {
    coll.object2 = pickup_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);
  return ord;
}

/*! Zero padding on fingertip links */
std::vector<arm_navigation_msgs::LinkPadding> 
GraspExecutor::linkPaddingForLift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  //return concat(MechanismInterface::fingertipPadding(pickup_goal.arm_name, 0.0), 
  //              pickup_goal.additional_link_padding);
  return concat(MechanismInterface::gripperPadding(pickup_goal.arm_name, 0.0), 
                pickup_goal.additional_link_padding);
}

GraspResult 
GraspExecutor::getInterpolatedIKForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
					const object_manipulation_msgs::Grasp &grasp,
					const std::vector<double> &grasp_joint_angles,
					trajectory_msgs::JointTrajectory &lift_trajectory)
{
  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.pose = grasp.grasp_pose;
  grasp_pose.header.frame_id = pickup_goal.target.reference_frame_id;
  grasp_pose.header.stamp = ros::Time(0);

  float actual_lift_distance;
  int error_code = 
    mechInterface().getInterpolatedIK(pickup_goal.arm_name, 
				      grasp_pose, 
				      pickup_goal.lift.direction, pickup_goal.lift.desired_distance,
				      grasp_joint_angles, 
				      grasp.grasp_posture, 
				      collisionOperationsForLift(pickup_goal), linkPaddingForLift(pickup_goal),
				      false, lift_trajectory, actual_lift_distance);
  ROS_DEBUG_NAMED("manipulation","  Lift distance: actual %f, min %f and desired %f", actual_lift_distance, pickup_goal.lift.min_distance,
           pickup_goal.lift.desired_distance);

  if (actual_lift_distance < pickup_goal.lift.min_distance)
  {
    ROS_DEBUG_NAMED("manipulation","  Lift trajectory  below min. threshold");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 0.0, 1.0); //blue
    if (lift_trajectory.points.empty())
    {
      ROS_DEBUG_NAMED("manipulation","  Lift trajectory empty; problem is with grasp location");
      if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) 
	return Result(GraspResult::GRASP_IN_COLLISION, true);
      else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
	return Result(GraspResult::GRASP_OUT_OF_REACH, true);
      else return Result(GraspResult::GRASP_UNFEASIBLE, true);
    }
    if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) 
      return Result(GraspResult::LIFT_IN_COLLISION, true);
    else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
      return Result(GraspResult::LIFT_OUT_OF_REACH, true);
    else return Result(GraspResult::LIFT_UNFEASIBLE, true);
  }

  // note: here we used to check if the last state is valid with default padding. That
  // check has been moved to prepareGrasp() so that it can be grouped together with other checks that
  // use similar planning scenes.
  
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
GraspExecutor::lift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  if (interpolated_lift_trajectory_.points.empty())
  {
    ROS_ERROR("  Grasp executor: lift trajectory not set");
    return Result(GraspResult::LIFT_FAILED, false);
  }

  //execute the unnormalized interpolated trajectory from grasp to lift
  mechInterface().attemptTrajectory(pickup_goal.arm_name, interpolated_lift_trajectory_, true);
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
GraspExecutor::checkAndExecuteGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
				    const object_manipulation_msgs::Grasp &grasp)
{
  //test if grasp is feasible and prepare trajectories  
  if (marker_publisher_)
  {
    geometry_msgs::PoseStamped marker_pose;
    marker_pose.pose = grasp.grasp_pose;
    marker_pose.header.frame_id = pickup_goal.target.reference_frame_id;
    marker_pose.header.stamp = ros::Time::now();
    marker_id_ = marker_publisher_->addGraspMarker(marker_pose);
  }  
  GraspResult result = prepareGrasp(pickup_goal, grasp);
  if (result.result_code != GraspResult::SUCCESS || pickup_goal.only_perform_feasibility_test) return result;

  result = executeGrasp(pickup_goal, grasp);
  if (result.result_code != GraspResult::SUCCESS) return result;

  //check if there is anything in gripper; if not, open gripper and retreat
  if (!mechInterface().graspPostureQuery(pickup_goal.arm_name, grasp))
  {
    ROS_DEBUG_NAMED("manipulation","Hand reports that grasp was not successfully executed; "
                    "releasing object and retreating");
    mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp,
                                    object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE, -1);    
    retreat(pickup_goal, grasp);
    return Result(GraspResult::GRASP_FAILED, false);
  }
  
  //attach object to gripper
  if (!pickup_goal.collision_object_name.empty())
  {
    mechInterface().attachObjectToGripper(pickup_goal.arm_name, pickup_goal.collision_object_name);
  }

  //lift the object
  result = lift(pickup_goal);
  if (result.result_code != GraspResult::SUCCESS) return result;

  return Result(GraspResult::SUCCESS, true);
}

} //namespace object_manipulator
