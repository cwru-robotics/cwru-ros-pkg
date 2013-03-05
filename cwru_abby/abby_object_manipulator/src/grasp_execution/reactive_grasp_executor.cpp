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

// Author(s): Kaijen Hsiao Matei Ciocarlie

#include "object_manipulator/grasp_execution/reactive_grasp_executor.h"

#include <object_manipulation_msgs/ReactiveGrasp.h>

//#include <demo_synchronizer/synchronizer_client.h>

#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"

using object_manipulation_msgs::GraspResult;

namespace object_manipulator {

GraspResult 
ReactiveGraspExecutor::executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
				    const object_manipulation_msgs::Grasp &grasp)
{
  //demo_synchronizer::getClient().sync(2, "Using motion planner to move arm to pre-grasp position");
  //demo_synchronizer::getClient().rviz(1, "Collision models;Environment contacts;Planning goal;Collision map");

  if ( !mechInterface().attemptMoveArmToGoal(pickup_goal.arm_name, 
					     interpolated_grasp_trajectory_.points.front().positions,
                                             pickup_goal.additional_collision_operations,
                                             pickup_goal.additional_link_padding) ) 
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: move_arm to pre-grasp reports failure");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 0.5, 0.0); //orange-ish
    return Result(GraspResult::MOVE_ARM_FAILED, true);
  }

  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                                         object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP, -1);
  
  geometry_msgs::PoseStamped final_grasp_pose_stamped;
  final_grasp_pose_stamped.pose = grasp.grasp_pose;
  final_grasp_pose_stamped.header.frame_id = pickup_goal.target.reference_frame_id;
  final_grasp_pose_stamped.header.stamp = ros::Time(0);

  object_manipulation_msgs::ReactiveGraspGoal reactive_grasp_goal;
  reactive_grasp_goal.arm_name = pickup_goal.arm_name;
  reactive_grasp_goal.target = pickup_goal.target;
  reactive_grasp_goal.final_grasp_pose = final_grasp_pose_stamped;
  reactive_grasp_goal.trajectory = interpolated_grasp_trajectory_;
  reactive_grasp_goal.collision_support_surface_name = pickup_goal.collision_support_surface_name;
  reactive_grasp_goal.pre_grasp_posture = grasp.pre_grasp_posture;
  reactive_grasp_goal.grasp_posture = grasp.grasp_posture;
  reactive_grasp_goal.max_contact_force = pickup_goal.max_contact_force;

  //demo_synchronizer::getClient().sync(2, "Calling fingertip reactive grasping module to execute grasp");
  //demo_synchronizer::getClient().rviz(1, "Collision models");

  //give the reactive grasp 3 minutes to do its thing
  ros::Duration timeout = ros::Duration(180.0);
  mechInterface().reactive_grasp_action_client_.client(pickup_goal.arm_name).sendGoal(reactive_grasp_goal);
  if ( !mechInterface().reactive_grasp_action_client_.client(pickup_goal.arm_name).waitForResult(timeout) )
  {
    ROS_ERROR("  Reactive grasp timed out");
    return Result(GraspResult::GRASP_FAILED, false);
  }
  object_manipulation_msgs::ReactiveGraspResult reactive_grasp_result = 
    *mechInterface().reactive_grasp_action_client_.client(pickup_goal.arm_name).getResult();
  if (reactive_grasp_result.manipulation_result.value != reactive_grasp_result.manipulation_result.SUCCESS)
  {
    ROS_ERROR("Reactive grasp failed with error code %d", reactive_grasp_result.manipulation_result.value);
    return Result(GraspResult::GRASP_FAILED, false);
  }
  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
ReactiveGraspExecutor::nonReactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.object1 = handDescription().gripperCollisionName(pickup_goal.arm_name);
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  //disable collisions between end-effector and table
  if (!pickup_goal.collision_support_surface_name.empty())
  {
    coll.object2 = pickup_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  //disable collisions between attached object and table
  if (!pickup_goal.collision_support_surface_name.empty() && !pickup_goal.collision_object_name.empty())
  {
    coll.object1 = pickup_goal.collision_object_name;
    coll.object2 = pickup_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);

  float actual_distance;
  if (!mechInterface().translateGripper(pickup_goal.arm_name, 
					pickup_goal.lift.direction, 
					ord, pickup_goal.additional_link_padding, 
					pickup_goal.lift.desired_distance, 0.0, actual_distance))
  {
    ROS_DEBUG_NAMED("manipulation","  Reactive grasp executor: lift performed no steps");
    return Result(GraspResult::LIFT_FAILED, false);
  }
  if (actual_distance < pickup_goal.lift.min_distance)
  {
    ROS_DEBUG_NAMED("manipulation","  Reactive grasp executor: lift distance below min threshold ");
    return Result(GraspResult::LIFT_FAILED, false);
  }
  if (actual_distance < pickup_goal.lift.desired_distance)
  {
    ROS_DEBUG_NAMED("manipulation","  Reactive grasp executor: lift distance below desired, but above min threshold");
  }
  else 
  {
    ROS_DEBUG_NAMED("manipulation","  Reactive grasp executor: desired lift distance executed");
  }
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
ReactiveGraspExecutor::reactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  object_manipulation_msgs::ReactiveLiftGoal reactive_lift_goal;
  reactive_lift_goal.lift = pickup_goal.lift;
  reactive_lift_goal.arm_name = pickup_goal.arm_name;
  reactive_lift_goal.target = pickup_goal.target;
  reactive_lift_goal.trajectory = interpolated_lift_trajectory_;
  reactive_lift_goal.collision_support_surface_name = pickup_goal.collision_support_surface_name;

  //demo_synchronizer::getClient().sync(2, "Calling fingertip reactive grasping module to execute lift");
  //demo_synchronizer::getClient().rviz(1, "Collision models");

  //give the reactive lift 1 minute to do its thing
  ros::Duration timeout = ros::Duration(60.0);
  mechInterface().reactive_lift_action_client_.client(pickup_goal.arm_name).sendGoal(reactive_lift_goal);
  if ( !mechInterface().reactive_lift_action_client_.client(pickup_goal.arm_name).waitForResult(timeout) )
  {
    ROS_ERROR("  Reactive lift timed out");
    return Result(GraspResult::LIFT_FAILED, false);
  }
  object_manipulation_msgs::ReactiveLiftResult reactive_lift_result = 
    *mechInterface().reactive_lift_action_client_.client(pickup_goal.arm_name).getResult();
  if (reactive_lift_result.manipulation_result.value != reactive_lift_result.manipulation_result.SUCCESS)
  {
    ROS_ERROR("Reactive lift failed with error code %d", reactive_lift_result.manipulation_result.value);
    return Result(GraspResult::LIFT_FAILED, false);
  }
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
ReactiveGraspExecutor::lift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  if (pickup_goal.use_reactive_lift)
  {
    return reactiveLift(pickup_goal);
  }
  else
  {
    return nonReactiveLift(pickup_goal);
  }
}

} //namespace object_manipulator
