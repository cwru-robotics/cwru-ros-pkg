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

#include "object_manipulator/grasp_execution/approach_lift_grasp.h"

#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/exceptions.h"
#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"

using object_manipulation_msgs::GraspResult;
using arm_navigation_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

//! One thousandth of a mm
float GraspTesterWithApproach::EPS = 1.0e-6;

// ---------------------------- Definitions ---------------------------------

void GraspTester::testGrasps(const object_manipulation_msgs::PickupGoal &goal,
                             const std::vector<object_manipulation_msgs::Grasp> &grasps,
                             std::vector<GraspExecutionInfo> &execution_info,
                             bool return_on_first_hit)
{
  execution_info.clear();
  for (size_t i=0; i<grasps.size(); i++)
  {
    GraspExecutionInfo info;
    ROS_DEBUG_NAMED("manipulation","Grasp tester: testing grasp %zd out of batch of %zd", i, grasps.size());
    if (feedback_function_) feedback_function_(i);
    if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();   
    if (marker_publisher_)
    {
      geometry_msgs::PoseStamped marker_pose;
      marker_pose.pose = grasps[i].grasp_pose;
      marker_pose.header.frame_id = goal.target.reference_frame_id;
      marker_pose.header.stamp = ros::Time::now();
      info.marker_id_ = marker_publisher_->addGraspMarker(marker_pose);
    }  
    testGrasp(goal, grasps[i], info);
    execution_info.push_back(info);
    if (info.result_.result_code == info.result_.SUCCESS && return_on_first_hit) return;
  }
}

void GraspPerformer::performGrasps(const object_manipulation_msgs::PickupGoal &goal,
                                   const std::vector<object_manipulation_msgs::Grasp> &grasps,
                                   std::vector<GraspExecutionInfo> &execution_info)
{
  for (size_t i=0; i<grasps.size(); i++)
  {
    if (feedback_function_) feedback_function_(i);
    if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
    if (i>= execution_info.size()) throw GraspException("Grasp Performer: not enough execution info provided");
    if (execution_info[i].result_.result_code != GraspResult::SUCCESS) continue;
    ROS_DEBUG_NAMED("manipulation","Grasp performer: trying grasp %zd out of batch of %zd", i, grasps.size());
    performGrasp(goal, grasps[i], execution_info[i]);
    if (execution_info[i].result_.result_code == execution_info[i].result_.SUCCESS || 
        !execution_info[i].result_.continuation_possible) return;
  }
}

// ------------------------------ Grasp Testers ----------------------------------

/*! Disables collision between gripper and target
 Disables ALL collisions on moved obstacles
*/
arm_navigation_msgs::OrderedCollisionOperations 
GraspTesterWithApproach::collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                                    const object_manipulation_msgs::Grasp &grasp)
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

  for (unsigned int i = 0; i < grasp.moved_obstacles.size(); i++)
  {
    ROS_DEBUG_NAMED("manipulation","  Disabling all collisions for lift against moved obstacle %s",
                     grasp.moved_obstacles[i].collision_name.c_str());
    coll.object1 = grasp.moved_obstacles[i].collision_name;
    //This disables all collisions with the object. Ignore possible collisions of the robot with the object 
    //during the push-grasp. We need all the capture region to be able to do this safely. Or we can disable 
    //collisions just with the forearm, which should work almost always but still theoretically is not the right 
    //thing to do.
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    ord.collision_operations.push_back(coll);
  }

  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);  
  return ord;
}

/*! Zero padding on gripper */
std::vector<arm_navigation_msgs::LinkPadding> 
GraspTesterWithApproach::linkPaddingForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                            const object_manipulation_msgs::Grasp&)
{
  return concat(MechanismInterface::gripperPadding(pickup_goal.arm_name, 0.0), 
                pickup_goal.additional_link_padding);
}

GraspResult 
GraspTesterWithApproach::getInterpolatedIKForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                                  const object_manipulation_msgs::Grasp &grasp,
                                                  const std::vector<double> &grasp_joint_angles,
                                                  GraspExecutionInfo &execution_info)
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
				      collisionOperationsForLift(pickup_goal, grasp), 
                                      linkPaddingForLift(pickup_goal, grasp),
				      false, execution_info.lift_trajectory_, actual_lift_distance);
  ROS_DEBUG_NAMED("manipulation","  Lift distance: actual %f, min %f and desired %f", 
                  actual_lift_distance, pickup_goal.lift.min_distance, pickup_goal.lift.desired_distance);

  if (actual_lift_distance < pickup_goal.lift.min_distance - EPS)
  {
    ROS_DEBUG_NAMED("manipulation","  Lift trajectory  below min. threshold");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 0.0, 0.0, 1.0);
    if (execution_info.lift_trajectory_.points.empty())
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
 
  return Result(GraspResult::SUCCESS, true);
}

/*! Disable collisions between end-effector and target
  Disables ALL collisions on moved obstacles 
*/
arm_navigation_msgs::OrderedCollisionOperations 
GraspTesterWithApproach::collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                                     const object_manipulation_msgs::Grasp &grasp)
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

  coll.object1 = pickup_goal.collision_object_name;
  //This disables all collisions with the object. Ignore possible collisions of the robot with the object 
  //during the push-grasp. We need all the capture region to be able to do this safely. Or we can disable 
  //collisions just with the forearm, which should work almost always but still theoretially is not the right 
  //thing to do.
  coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
  ord.collision_operations.push_back(coll);

  for (unsigned int i = 0; i < grasp.moved_obstacles.size(); i++)
  {
    ROS_DEBUG_NAMED("manipulation","  Disabling all collisions for grasp against moved obstacle %s",
                     grasp.moved_obstacles[i].collision_name.c_str());
    coll.object1 = grasp.moved_obstacles[i].collision_name;
    //This disables all collisions with the object. Ignore possible collisions of the robot with the object 
    //during the push-grasp. We need all the capture region to be able to do this safely. Or we can disable 
    //collisions just with the forearm, which should work almost always but still theoretially is not the right 
    //thing to do.
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    ord.collision_operations.push_back(coll);
  }

  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);
  return ord;
}

/*! Zero padding on gripper links */
std::vector<arm_navigation_msgs::LinkPadding> 
GraspTesterWithApproach::linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                             const object_manipulation_msgs::Grasp&)
{
  return concat(MechanismInterface::gripperPadding(pickup_goal.arm_name, 0.0), 
                pickup_goal.additional_link_padding);
}

GraspResult 
GraspTesterWithApproach::getInterpolatedIKForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                                   const object_manipulation_msgs::Grasp &grasp,
                                                   GraspExecutionInfo &execution_info)
{
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

  std::vector<double> empty;
  //remember to pass that we want to flip the trajectory
  float actual_approach_distance;
  int error_code = mechInterface().getInterpolatedIK(pickup_goal.arm_name, 
						     grasp_pose_stamped, 
						     direction, grasp.desired_approach_distance,
						     empty, 
						     grasp.pre_grasp_posture,
						     collisionOperationsForGrasp(pickup_goal, grasp), 
						     linkPaddingForGrasp(pickup_goal, grasp),
						     true, execution_info.approach_trajectory_, 
                                                     actual_approach_distance);
  ROS_DEBUG_NAMED("manipulation","  Grasp executor approach distance: actual (%f), min(%f) and desired (%f)", 
            actual_approach_distance, grasp.min_approach_distance, grasp.desired_approach_distance);

  if ( actual_approach_distance < grasp.min_approach_distance - EPS)
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: interpolated IK for grasp below min threshold");
    if (execution_info.approach_trajectory_.points.empty())
    {
      ROS_DEBUG_NAMED("manipulation","  Grasp executor: interpolated IK empty, problem is with grasp location");
      if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 1.0, 1.0, 0.0); //yellow
      if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) 
	return Result(GraspResult::GRASP_IN_COLLISION, true);
      else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
	return Result(GraspResult::GRASP_OUT_OF_REACH, true);
      else return Result(GraspResult::GRASP_UNFEASIBLE, true);      
    }
    if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 0.0, 1.0, 1.0); //cyan
    if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) 
      return Result(GraspResult::PREGRASP_IN_COLLISION, true);
    else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
      return Result(GraspResult::PREGRASP_OUT_OF_REACH, true);
    else return Result(GraspResult::PREGRASP_UNFEASIBLE, true);      
  }

  return Result(GraspResult::SUCCESS, true);
}

void GraspTesterWithApproach::testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                        const object_manipulation_msgs::Grasp &grasp,
                                        GraspExecutionInfo &execution_info)
{
  if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 1.0, 0.0, 0.0); //red

  execution_info.result_ = getInterpolatedIKForGrasp(pickup_goal, grasp, execution_info);
  if ( execution_info.result_.result_code != GraspResult::SUCCESS )
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: failed to generate approach trajectory");
    return;
  }

  //check for lift trajectory starting from grasp solution
  std::vector<double> grasp_joint_angles = execution_info.approach_trajectory_.points.back().positions;
  execution_info.result_ = getInterpolatedIKForLift(pickup_goal, grasp, grasp_joint_angles, execution_info);
  if (execution_info.result_.result_code != GraspResult::SUCCESS)
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: failed to generate lift trajectory");
    return;
  }

  //check if the first pose in grasp trajectory is valid
  //when we check from pre-grasp to grasp we use custom link padding, so we need to check here
  //if the initial pose is feasible with default padding; otherwise, move_arm might refuse to 
  //take us there
  if ( !mechInterface().checkStateValidity(pickup_goal.arm_name, 
                                           execution_info.approach_trajectory_.points.front().positions,
                                           pickup_goal.additional_collision_operations,
                                           pickup_goal.additional_link_padding) )
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: initial pose in grasp trajectory is unfeasible "
                    "with default padding");
    execution_info.result_ = Result(GraspResult::PREGRASP_UNFEASIBLE, true);
    return;
  }
  ROS_DEBUG_NAMED("manipulation","  Grasp executor: goal pose for move arm passed validity test");

  //check if the last pose in lift trajectory is valid
  //when we check for lift we use custom link padding, so we need to check here if the last pose 
  //is feasible with default padding; otherwise, move_arm might refuse to take us out of there
  if ( pickup_goal.lift.min_distance != 0 && 
       !mechInterface().checkStateValidity(pickup_goal.arm_name, 
                                           execution_info.lift_trajectory_.points.back().positions,
                                           collisionOperationsForLift(pickup_goal, grasp),
                                           pickup_goal.additional_link_padding) )
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: last pose in lift trajectory is unfeasible with default padding");
    execution_info.result_ = Result(GraspResult::LIFT_UNFEASIBLE, true);
    return;
  }  
  ROS_DEBUG_NAMED("manipulation","  Grasp executor: final lift pose passed validity test");
}

/*! Disable collisions between everything and everything */
arm_navigation_msgs::OrderedCollisionOperations 
UnsafeGraspTester::collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                               const object_manipulation_msgs::Grasp&)
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
UnsafeGraspTester::collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                              const object_manipulation_msgs::Grasp&)
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


// ---------------------------- Grasp Performers ---------------------------------

void StandardGraspPerformer::performGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                          const object_manipulation_msgs::Grasp &grasp,
                                          GraspExecutionInfo &execution_info)
{
  execution_info.result_ = approachAndGrasp(pickup_goal, grasp, execution_info);
  if (execution_info.result_.result_code != GraspResult::SUCCESS) return;

  //check if there is anything in gripper; if not, open gripper and retreat
  if (!mechInterface().graspPostureQuery(pickup_goal.arm_name, grasp))
  {
    ROS_DEBUG_NAMED("manipulation","Hand reports that grasp was not successfully executed; "
                    "releasing object and retreating");
    mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp,
                                object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE, -1);    
    retreat(pickup_goal, grasp, execution_info);
    execution_info.result_ = Result(GraspResult::GRASP_FAILED, false);
    return;
  }
  
  //attach object to gripper
  if (!pickup_goal.collision_object_name.empty())
  {
    mechInterface().attachObjectToGripper(pickup_goal.arm_name, pickup_goal.collision_object_name);
  }

  execution_info.result_ = lift(pickup_goal, grasp, execution_info);
}


object_manipulation_msgs::GraspResult 
StandardGraspPerformer::approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                         const object_manipulation_msgs::Grasp &grasp,
                                         GraspExecutionInfo &execution_info)
{
  ROS_INFO("Additional collision operations:");
  for (unsigned int i = 0; i < pickup_goal.additional_collision_operations.collision_operations.size(); i++)
  {
    ROS_INFO_STREAM("\t for objects " << pickup_goal.additional_collision_operations.collision_operations[i].object1 << " and " << pickup_goal.additional_collision_operations.collision_operations[i].object2 );
  }
  if ( !mechInterface().attemptMoveArmToGoal(pickup_goal.arm_name, 
					     execution_info.approach_trajectory_.points.front().positions,
                                             pickup_goal.additional_collision_operations,
                                             pickup_goal.additional_link_padding) ) 
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: move_arm to pre-grasp reports failure");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 1.0, 0.5, 0.0); //orange-ish
    return Result(GraspResult::MOVE_ARM_FAILED, true);
  }

  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                     object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP, -1);

  mechInterface().attemptTrajectory(pickup_goal.arm_name, execution_info.approach_trajectory_, true);

  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                     object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP, pickup_goal.max_contact_force);    

  if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

object_manipulation_msgs::GraspResult 
StandardGraspPerformer::lift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                             const object_manipulation_msgs::Grasp &grasp,
                             GraspExecutionInfo &execution_info)
{
  if (execution_info.lift_trajectory_.points.empty())
  {
    ROS_ERROR("  Grasp executor: lift trajectory not set");
    return Result(GraspResult::LIFT_FAILED, false);
  }
  mechInterface().attemptTrajectory(pickup_goal.arm_name, execution_info.lift_trajectory_, true);
  return Result(GraspResult::SUCCESS, true);

}

object_manipulation_msgs::GraspResult 
StandardGraspPerformer::retreat(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                const object_manipulation_msgs::Grasp &grasp,
                                GraspExecutionInfo &execution_info)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  //disable collision between gripper and object
  coll.object1 = handDescription().gripperCollisionName(pickup_goal.arm_name);
  coll.object2 = pickup_goal.collision_object_name;
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  ord.collision_operations.push_back(coll);
  //disable collision between gripper and table
  coll.object2 = pickup_goal.collision_support_surface_name;
  ord.collision_operations.push_back(coll);
  ord.collision_operations = concat(ord.collision_operations, 
                                    pickup_goal.additional_collision_operations.collision_operations);

  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = handDescription().gripperFrame(pickup_goal.arm_name);
  direction.vector = mechInterface().negate( handDescription().approachDirection(pickup_goal.arm_name) );

  //even if the complete retreat trajectory is not possible, execute as many
  //steps as we can (pass min_distance = 0)
  float retreat_distance = grasp.min_approach_distance;
  float actual_distance;
  if (!mechInterface().translateGripper(pickup_goal.arm_name, direction,
					ord, pickup_goal.additional_link_padding, 
                                        retreat_distance, 0, actual_distance))
  {
    ROS_ERROR(" Grasp executor: failed to retreat gripper at all");
    return Result(GraspResult::RETREAT_FAILED, false);
  }
  if (actual_distance < retreat_distance)
  {
    ROS_WARN(" Grasp executor: only partial retreat (%f) succeeeded", actual_distance);
    return Result(GraspResult::RETREAT_FAILED, false);
  }
  return Result(GraspResult::SUCCESS, true);
}

object_manipulation_msgs::GraspResult 
ReactiveGraspPerformer::nonReactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                        const object_manipulation_msgs::Grasp &grasp,
                                        GraspExecutionInfo &execution_info)
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

object_manipulation_msgs::GraspResult 
ReactiveGraspPerformer::reactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                     const object_manipulation_msgs::Grasp &grasp,
                                     GraspExecutionInfo &execution_info)
{
  object_manipulation_msgs::ReactiveLiftGoal reactive_lift_goal;
  reactive_lift_goal.lift = pickup_goal.lift;
  reactive_lift_goal.arm_name = pickup_goal.arm_name;
  reactive_lift_goal.target = pickup_goal.target;
  reactive_lift_goal.trajectory = execution_info.lift_trajectory_;
  reactive_lift_goal.collision_support_surface_name = pickup_goal.collision_support_surface_name;

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

object_manipulation_msgs::GraspResult 
ReactiveGraspPerformer::lift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                             const object_manipulation_msgs::Grasp &grasp,
                             GraspExecutionInfo &execution_info)
{
  if (pickup_goal.use_reactive_lift)
  {
    return reactiveLift(pickup_goal, grasp, execution_info);
  }
  else
  {
    return nonReactiveLift(pickup_goal, grasp, execution_info);
  }
}

object_manipulation_msgs::GraspResult 
ReactiveGraspPerformer::approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                         const object_manipulation_msgs::Grasp &grasp,
                                         GraspExecutionInfo &execution_info)
{
  if ( !mechInterface().attemptMoveArmToGoal(pickup_goal.arm_name, 
					     execution_info.approach_trajectory_.points.front().positions,
                                             pickup_goal.additional_collision_operations,
                                             pickup_goal.additional_link_padding) ) 
  {
    ROS_DEBUG_NAMED("manipulation","  Grasp executor: move_arm to pre-grasp reports failure");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 1.0, 0.5, 0.0); //orange-ish
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
  reactive_grasp_goal.trajectory = execution_info.approach_trajectory_;
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
  if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

object_manipulation_msgs::GraspResult 
UnsafeGraspPerformer::approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                       const object_manipulation_msgs::Grasp &grasp,
                                       GraspExecutionInfo &execution_info)
{
  ROS_DEBUG_NAMED("manipulation", "executing unsafe grasp");

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
  geometry_msgs::PoseStamped pregrasp_pose = mechInterface().translateGripperPose(direction_norm, grasp_pose_stamped, 
                                                                                  pickup_goal.arm_name);
  
  //move to the pre-grasp using the Cartesian controller
  mechInterface().moveArmToPoseCartesian(pickup_goal.arm_name, pregrasp_pose, ros::Duration(15.0), 
                                         .0015, .01, 0.02, 0.16, 0.005, 0.087, 0.1, 
                                         execution_info.approach_trajectory_.points.front().positions);

  //move to the pre-grasp hand posture
  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp, 
                           object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP, -1);

  //if not using reactive grasping, execute the unnormalized interpolated trajectory from pre-grasp to grasp
  if(!pickup_goal.use_reactive_execution)
  {  
    mechInterface().attemptTrajectory(pickup_goal.arm_name, execution_info.approach_trajectory_, true);
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
    reactive_grasp_goal.trajectory = execution_info.approach_trajectory_;
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
  if (marker_publisher_) marker_publisher_->colorGraspMarker(execution_info.marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

}
