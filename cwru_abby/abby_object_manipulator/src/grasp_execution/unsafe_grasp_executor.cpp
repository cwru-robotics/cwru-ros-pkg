/*********************************************************************
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#include "object_manipulator/grasp_execution/unsafe_grasp_executor.h"
#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"
#include "object_manipulator/tools/mechanism_interface.h"

using object_manipulation_msgs::GraspResult;

namespace object_manipulator {

GraspResult 
UnsafeGraspExecutor::executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
					const object_manipulation_msgs::Grasp &grasp)
{
  ROS_INFO("executing unsafe grasp");

  //compute the pre-grasp pose
  //get the grasp pose in the right frame
  geometry_msgs::PoseStamped grasp_pose_stamped;
  grasp_pose_stamped.pose = grasp.grasp_pose;
  grasp_pose_stamped.header.frame_id = pickup_goal.target.reference_frame_id;
  grasp_pose_stamped.header.stamp = ros::Time(0);

  //use the opposite of the approach direction as we are going backwards, from grasp to pre-grasp
  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = handDescription().gripperFrame(pickup_goal.arm_name);
  direction.vector = mechInterface().negate( handDescription().approachDirection(pickup_goal.arm_name) );

  //make sure the input is normalized
  geometry_msgs::Vector3Stamped direction_norm = direction;
  direction_norm.vector = mechInterface().normalize(direction.vector);

  //multiply the approach direction by the desired length and translate the grasp pose back along it
  double desired_trajectory_length = fabs(grasp.desired_approach_distance);
  direction_norm.vector.x *= desired_trajectory_length;
  direction_norm.vector.y *= desired_trajectory_length;
  direction_norm.vector.z *= desired_trajectory_length;
  geometry_msgs::PoseStamped pregrasp_pose = mechInterface().translateGripperPose(direction_norm, grasp_pose_stamped, pickup_goal.arm_name);
  
  //move to the pre-grasp using the Cartesian controller
  mechInterface().moveArmToPoseCartesian(pickup_goal.arm_name, pregrasp_pose, ros::Duration(15.0), 
		  .0015, .01, 0.02, 0.16, 0.005, 0.087, 0.1, 
		  interpolated_grasp_trajectory_.points[interpolated_grasp_trajectory_.points.size()-1].positions);

  //move to the pre-grasp hand posture
  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                                         object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP, -1);

  //if not using reactive grasping, execute the unnormalized interpolated trajectory from pre-grasp to grasp
  if(!pickup_goal.use_reactive_execution)
  {  
    mechInterface().attemptTrajectory(pickup_goal.arm_name, interpolated_grasp_trajectory_, true);
    mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                                           object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP, pickup_goal.max_contact_force);    
  }

  //otherwise, call reactive grasping
  else
  {
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
  }

  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}


/*! Disable collisions between everything and everything */
arm_navigation_msgs::OrderedCollisionOperations 
UnsafeGraspExecutor::collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  ord.collision_operations.push_back(coll);
  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);
  return ord;
}

/*! Disables collision between everything and everything */
arm_navigation_msgs::OrderedCollisionOperations 
UnsafeGraspExecutor::collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  ord.collision_operations.push_back(coll);
  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);
  return ord;
}

} //namespace object_manipulator
