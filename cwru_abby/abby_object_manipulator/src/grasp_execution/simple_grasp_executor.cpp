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

#include "object_manipulator/grasp_execution/simple_grasp_executor.h"

//#include <demo_synchronizer/synchronizer_client.h>

using object_manipulation_msgs::GraspResult;

namespace object_manipulator {

GraspResult SimpleGraspExecutor::prepareGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
					      const object_manipulation_msgs::Grasp &grasp)
{
  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 0.0, 0.0); //red

  geometry_msgs::PoseStamped grasp_pose;
  grasp_pose.pose = grasp.grasp_pose;
  grasp_pose.header.frame_id = pickup_goal.target.reference_frame_id;
  grasp_pose.header.stamp = ros::Time(0);

  //demo_synchronizer::getClient().sync(3, "Checking IK solution for final grasp position and interpolated IK for lift");
  //demo_synchronizer::getClient().rviz(3, "Collision models;Interpolated IK;IK contacts;Grasp execution");

  if ( !mechInterface().getIKForPose(pickup_goal.arm_name, grasp_pose, ik_response_,
                                     pickup_goal.additional_collision_operations,
                                     pickup_goal.additional_link_padding) ) 
  {
    ROS_DEBUG_STREAM("  Grasp execution: initial IK check failed");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 1.0, 0.0); //yellow
    return Result(GraspResult::GRASP_UNFEASIBLE, true);
  }
  ROS_DEBUG_NAMED("manipulation","  Grasp executor: IK for grasp succeeded.");

  std::vector<double> grasp_joint_angles = ik_response_.solution.joint_state.position;
  GraspResult result = getInterpolatedIKForLift(pickup_goal, grasp, grasp_joint_angles, interpolated_lift_trajectory_);
  if (result.result_code != GraspResult::SUCCESS) return result;

  //check if the last pose in lift trajectory is valid
  //when we check for lift we use custom link padding, so we need to check here if the last pose 
  //is feasible with default padding; otherwise, move_arm might refuse to take us out of there
  if ( pickup_goal.lift.min_distance != 0 && 
       !mechInterface().checkStateValidity(pickup_goal.arm_name, interpolated_lift_trajectory_.points.back().positions,
                                           collisionOperationsForLift(pickup_goal),
                                           pickup_goal.additional_link_padding) )
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: last pose in lift trajectory is unfeasible with default padding");
    return Result(GraspResult::LIFT_UNFEASIBLE, true);
  }

  ROS_DEBUG_NAMED("manipulation","  Grasp executor: interpolated IK for lift succeeded.");  
  return Result(GraspResult::SUCCESS, true);
}

GraspResult 
SimpleGraspExecutor::executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
				  const object_manipulation_msgs::Grasp &grasp)
{
  //demo_synchronizer::getClient().sync(2, "Using motion planner to move the arm to grasp position");
  //demo_synchronizer::getClient().rviz(1, "Collision models;Planning goal;Collision map;Environment contacts");

  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                                         object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP, -1);

  if ( !mechInterface().attemptMoveArmToGoal(pickup_goal.arm_name, ik_response_.solution.joint_state.position,
                                             pickup_goal.additional_collision_operations,
                                             pickup_goal.additional_link_padding) ) 
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp execution: move arm reports failure");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 0.5, 0.0); //orange-ish
    return Result(GraspResult::MOVE_ARM_FAILED, true);
  }

  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                                         object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP, pickup_goal.max_contact_force);    
  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

} //namespace object_manipulator
