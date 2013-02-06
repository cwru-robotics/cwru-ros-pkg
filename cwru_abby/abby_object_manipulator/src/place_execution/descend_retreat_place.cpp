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

#include <object_manipulator/place_execution/descend_retreat_place.h>

#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/exceptions.h"
#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"

using arm_navigation_msgs::ArmNavigationErrorCodes;
using object_manipulation_msgs::PlaceLocationResult;

namespace object_manipulator {

//! One thousandth of a mm
float StandardPlaceTester::EPS = 1.0e-6;

// ---------------------------- Definitions ---------------------------------

void PlaceTester::testPlaces(const object_manipulation_msgs::PlaceGoal &goal,
                             const std::vector<geometry_msgs::PoseStamped> &place_locations,
                             std::vector<PlaceExecutionInfo> &execution_info,
                             bool return_on_first_hit)
{
  execution_info.clear();
  for (size_t i=0; i<place_locations.size(); i++)
  {
    ROS_DEBUG_NAMED("manipulation","Place tester: testing place %zd out of batch of %zd", i, place_locations.size());
    if (feedback_function_) feedback_function_(i);
    if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
    PlaceExecutionInfo info;

    //compute gripper location for final place
    info.gripper_place_pose_ = computeGripperPose(place_locations[i], goal.grasp.grasp_pose, 
                                                  handDescription().robotFrame(goal.arm_name));
    if (marker_publisher_)
    {
      info.marker_id_ = marker_publisher_->addGraspMarker(info.gripper_place_pose_);
      marker_publisher_->colorGraspMarker(info.marker_id_, 1.0, 0.0, 1.0); //magenta
    }
    testPlace(goal, place_locations[i], info);
    execution_info.push_back(info);
    if (info.result_.result_code == info.result_.SUCCESS && return_on_first_hit) return;
  }
}

geometry_msgs::PoseStamped 
PlaceTester::computeGripperPose(geometry_msgs::PoseStamped place_location, 
                                        geometry_msgs::Pose grasp_pose, 
                                        std::string frame_id)
{
  //get the gripper pose relative to place location
  tf::Transform place_trans;
  tf::poseMsgToTF(place_location.pose, place_trans);
  tf::Transform grasp_trans;
  tf::poseMsgToTF(grasp_pose, grasp_trans);
  grasp_trans = place_trans * grasp_trans;

  //get it in the requested frame
  tf::Stamped<tf::Pose> grasp_trans_stamped;
  grasp_trans_stamped.setData(grasp_trans);
  grasp_trans_stamped.frame_id_ = place_location.header.frame_id;
  grasp_trans_stamped.stamp_ = ros::Time::now();
  if (!listener_.waitForTransform(frame_id, place_location.header.frame_id, ros::Time::now(), ros::Duration(1.0)))
  {
    ROS_ERROR("Object place: tf does not have transform from %s to %s", 
	      place_location.header.frame_id.c_str(),
	      frame_id.c_str());
    throw MechanismException( std::string("Object place: tf does not have transform from ") + 
			      place_location.header.frame_id.c_str() + std::string(" to ") + 
			      frame_id);
  }
  tf::Stamped<tf::Pose> grasp_trans_base;
  try
  {
    listener_.transformPose( frame_id, grasp_trans_stamped, grasp_trans_base);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Object place: tf failed to transform gripper pose into frame %s; exception: %s", 
	      frame_id.c_str(), ex.what());
    throw MechanismException( std::string("tf failed to transform gripper pose into frame ") + 
			      frame_id + std::string("; tf exception: ") +  
			      std::string(ex.what()) );
  }
  
  geometry_msgs::PoseStamped gripper_pose;
  tf::poseTFToMsg(grasp_trans_base, gripper_pose.pose);
  gripper_pose.header.frame_id = frame_id;
  gripper_pose.header.stamp = ros::Time::now();
  return gripper_pose;
}


void PlacePerformer::performPlaces(const object_manipulation_msgs::PlaceGoal &goal,
                                   const std::vector<geometry_msgs::PoseStamped> &place_locations,
                                   std::vector<PlaceExecutionInfo> &execution_info)
{
  for (size_t i=0; i<place_locations.size(); i++)
  {
    if (feedback_function_) feedback_function_(i);
    if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
    if (i>= execution_info.size()) throw GraspException("Place Performer: not enough execution info provided");
    if (execution_info[i].result_.result_code != PlaceLocationResult::SUCCESS) continue;
    ROS_DEBUG_NAMED("manipulation","Place performer: trying place %zd out of batch of %zd", i, place_locations.size());
    performPlace(goal, place_locations[i], execution_info[i]);
    if (execution_info[i].result_.result_code == execution_info[i].result_.SUCCESS || 
        !execution_info[i].result_.continuation_possible) return;
  }
}

// ---------------------------- Testers ---------------------------------

void StandardPlaceTester::testPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                                    const geometry_msgs::PoseStamped &place_location,
                                    PlaceExecutionInfo &execution_info)
{
  //disable collisions between grasped object and table
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  if (!place_goal.collision_object_name.empty() && !place_goal.collision_support_surface_name.empty())
  {
    coll.object1 = place_goal.collision_object_name;
    coll.object2 = place_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  if (place_goal.allow_gripper_support_collision)
  {
    coll.object1 = handDescription().gripperCollisionName(place_goal.arm_name);
    coll.object2 = place_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  ord.collision_operations = concat(ord.collision_operations, 
                                    place_goal.additional_collision_operations.collision_operations);

  //zero padding on fingertip links
  //std::vector<arm_navigation_msgs::LinkPadding> link_padding = 
  //  MechanismInterface::fingertipPadding(place_goal.arm_name, 0.0);
  std::vector<arm_navigation_msgs::LinkPadding> link_padding = 
    MechanismInterface::gripperPadding(place_goal.arm_name, 0.0);

  // padding on grasped object, which is still attached to the gripper
  arm_navigation_msgs::LinkPadding padding;
  padding.link_name = handDescription().attachedName(place_goal.arm_name);
  padding.padding = place_goal.place_padding;
  link_padding.push_back(padding);
  link_padding = concat(link_padding, place_goal.additional_link_padding);

  geometry_msgs::Vector3Stamped place_direction;
  place_direction.header.frame_id = place_goal.approach.direction.header.frame_id;
  place_direction.header.stamp = ros::Time::now();
  place_direction.vector = mechInterface().negate(place_goal.approach.direction.vector);

  //search backwards from place to pre-place
  std::vector<double> empty;
  float actual_distance;
  int error_code = mechInterface().getInterpolatedIK(place_goal.arm_name, 
						     execution_info.gripper_place_pose_, place_direction,
						     place_goal.approach.desired_distance,
						     empty, 
						     place_goal.grasp.grasp_posture, 
						     ord, link_padding, 
						     true, execution_info.descend_trajectory_, actual_distance);
  ROS_DEBUG_NAMED("manipulation"," Place trajectory: actual(%f), min(%f), desired (%f)", 
            actual_distance, place_goal.approach.min_distance, place_goal.approach.desired_distance);

  if (actual_distance < place_goal.approach.min_distance - EPS)
  {
    ROS_DEBUG_NAMED("manipulation","Place trajectory below min. threshold");
    if (execution_info.descend_trajectory_.points.empty())
    {
      ROS_DEBUG_NAMED("manipulation","Place trajectory empty; problem is with place location");
      if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) {
	execution_info.result_ = Result(PlaceLocationResult::PLACE_IN_COLLISION, true);
        return;
      }
      else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED) {
	execution_info.result_ = Result(PlaceLocationResult::PLACE_OUT_OF_REACH, true);
        return;
      }
      else {
        execution_info.result_ = Result(PlaceLocationResult::PLACE_UNFEASIBLE, true);      
        return;
      }
    }
    if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) {
      execution_info.result_ = Result(PlaceLocationResult::PREPLACE_IN_COLLISION, true);
      return;
    }
    else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED) {
      execution_info.result_ = Result(PlaceLocationResult::PREPLACE_OUT_OF_REACH, true);
      return;
    }
    else {
      execution_info.result_ = Result(PlaceLocationResult::PREPLACE_UNFEASIBLE, true);      
      return;
    }
  }


  //make sure first position is feasible with default padding
  if ( !mechInterface().checkStateValidity(place_goal.arm_name, 
                                           execution_info.descend_trajectory_.points.front().positions,
                                           place_goal.additional_collision_operations,
                                           place_goal.additional_link_padding) )
  {
    ROS_DEBUG_NAMED("manipulation","First pose in place trajectory is unfeasible with default padding");
    execution_info.result_ = Result(PlaceLocationResult::PREPLACE_UNFEASIBLE, true);
    return;
  }
  

  ord.collision_operations.clear();
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  //disable all collisions on grasped object, since we are no longer holding it during the retreat
  if (!place_goal.collision_object_name.empty())
  {
    coll.object1 = place_goal.collision_object_name;
    coll.object2 = coll.COLLISION_SET_ALL;
    ord.collision_operations.push_back(coll);
  }
  if (place_goal.allow_gripper_support_collision)
  {
    coll.object1 = handDescription().gripperCollisionName(place_goal.arm_name);
    coll.object2 = place_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  ord.collision_operations = concat(ord.collision_operations, 
                                    place_goal.additional_collision_operations.collision_operations);

  geometry_msgs::Vector3Stamped retreat_direction;
  retreat_direction.header.stamp = ros::Time::now();
  retreat_direction.header.frame_id = handDescription().gripperFrame(place_goal.arm_name);
  retreat_direction.vector = mechInterface().negate( handDescription().approachDirection(place_goal.arm_name) );

  //search from place to retreat, using solution from place as seed
  std::vector<double> place_joint_angles = execution_info.descend_trajectory_.points.back().positions;  
  mechInterface().getInterpolatedIK(place_goal.arm_name, 
				    execution_info.gripper_place_pose_, retreat_direction,
				    place_goal.desired_retreat_distance,
				    place_joint_angles, 
				    place_goal.grasp.pre_grasp_posture, 
				    ord, link_padding, 
				    false, execution_info.retreat_trajectory_, actual_distance);
  ROS_DEBUG_NAMED("manipulation","Retreat trajectory: actual (%f), min (%f) and desired (%f)", actual_distance,
	   place_goal.min_retreat_distance, place_goal.desired_retreat_distance);

  if (actual_distance < place_goal.min_retreat_distance - EPS)
  {
   ROS_DEBUG_NAMED("manipulation","Retreat trajectory below min. threshold");
    if (execution_info.retreat_trajectory_.points.empty())
    {
      ROS_DEBUG_NAMED("manipulation","Retreat trajectory empty; problem is with place location");
      if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) {
	execution_info.result_ = Result(PlaceLocationResult::PLACE_IN_COLLISION, true);
        return;
      }
      else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED) {
	execution_info.result_ = Result(PlaceLocationResult::PLACE_OUT_OF_REACH, true);
        return;
      }
      else {
        execution_info.result_ = Result(PlaceLocationResult::PLACE_UNFEASIBLE, true);      
        return;
      }
    }
    if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED) {
      execution_info.result_ = Result(PlaceLocationResult::RETREAT_IN_COLLISION, true);
      return;
    }
    else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED) {
      execution_info.result_ = Result(PlaceLocationResult::RETREAT_OUT_OF_REACH, true);
      return;
    }
    else {
      execution_info.result_ = Result(PlaceLocationResult::RETREAT_UNFEASIBLE, true);  
      return;
    }
  }
  execution_info.result_ = Result(PlaceLocationResult::SUCCESS, true);
}

// ---------------------------- Performers ---------------------------------

/*! This function will re-compute the trajectory to retreat. Note that, before we even
  begun placing, we did compute a retreat trajectory, so we could maybe use that one. 
  However, we are assuming that maybe the placing trajectory did not leave us precisely
  in the desired spot, so maybe that retreat trajectory is no longer valid.

  Here we attempt to re-compute a retreat trajectory, by disabling contact between the
  gripper and the target object. We also disable collision between the gripper and the 
  table, as we assume a retreat trajectory can never bring the gripper in collision 
  with the table (or if it does, the wrist will hit too and we'll catch it). We then
  execute this trajectory however many points it has.

  The fact that a retreat trajectory was found before even placing should make it very
  likely (but not 100% sure) that we will find one now as well.
*/
PlaceLocationResult StandardPlacePerformer::retreat(const object_manipulation_msgs::PlaceGoal &place_goal)
{
  arm_navigation_msgs::OrderedCollisionOperations ord;
  arm_navigation_msgs::CollisionOperation coll;
  coll.object1 = handDescription().gripperCollisionName(place_goal.arm_name);
  coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
  //disable collision between gripper and object
  if (!place_goal.collision_object_name.empty())
  {
    coll.object2 = place_goal.collision_object_name;
    ord.collision_operations.push_back(coll);
  }
  //disable collision between gripper and table
  if (!place_goal.collision_support_surface_name.empty()) 
  {
    coll.object2 = place_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  ord.collision_operations = concat(ord.collision_operations, 
                                    place_goal.additional_collision_operations.collision_operations);

  //zero padding on gripper; shouldn't matter much due to collisions disabled above
  std::vector<arm_navigation_msgs::LinkPadding> link_padding 
    = concat(MechanismInterface::gripperPadding(place_goal.arm_name, 0.0),
             place_goal.additional_link_padding);

  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = handDescription().gripperFrame(place_goal.arm_name);
  direction.vector = mechInterface().negate( handDescription().approachDirection(place_goal.arm_name) );

  float actual_distance;
  mechInterface().translateGripper(place_goal.arm_name, 
				   direction, 
				   ord, link_padding, 
				   place_goal.desired_retreat_distance, 0, actual_distance);
  if (actual_distance < place_goal.min_retreat_distance)
  {
    ROS_DEBUG_NAMED("manipulation","Object place: retreat incomplete (%f executed and %f desired)", 
	     actual_distance, place_goal.min_retreat_distance);
    return Result(PlaceLocationResult::RETREAT_FAILED, false);
  }

  return Result(PlaceLocationResult::SUCCESS, true);
}

/*! In the default implementation, this simply executes the interpolated trajectory from
  pre-place to place that has already been computed by a place tester.
*/
PlaceLocationResult 
StandardPlacePerformer::placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
                                      const PlaceExecutionInfo &execution_info)
{
  mechInterface().attemptTrajectory(place_goal.arm_name, execution_info.descend_trajectory_, true);
  return Result(PlaceLocationResult::SUCCESS, true);
}


void StandardPlacePerformer::performPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                                          const geometry_msgs::PoseStamped &place_location,
                                          PlaceExecutionInfo &execution_info)
{
  //whether we are using the constraints or not
  bool use_constraints = true;
  //check if we can actually understand the constraints
  if(!constraintsUnderstandable(place_goal.path_constraints))
  {
    ROS_WARN("Constraints passed to place executor are of types not yet handled. Ignoring them.");
    use_constraints = false;
  }
  if (place_goal.path_constraints.orientation_constraints.empty())
  {
    use_constraints = false;
  }  
  if(use_constraints)
  {
    //recompute the pre-place pose from the already computed trajectory
    geometry_msgs::PoseStamped place_pose;
    place_pose.header.frame_id = place_location.header.frame_id;
    place_pose.header.stamp = ros::Time(0.0);
    //ROS_DEBUG_NAMED("manipulation","Place pose in frame: %s",place_pose.header.frame_id.c_str());
    if(!mechInterface().getFK(place_goal.arm_name, execution_info.descend_trajectory_.points.front().positions, 
                              place_pose))
    {
      ROS_ERROR("Could not re-compute pre-place pose based on trajectory");
      throw MechanismException("Could not re-compute pre-place pose based on trajectory");       
    }
    ROS_DEBUG_NAMED("manipulation","Attempting move arm to pre-place with constraints");
    if (!mechInterface().moveArmConstrained(place_goal.arm_name, place_pose, 
                                            place_goal.additional_collision_operations,
                                            place_goal.additional_link_padding,
                                            place_goal.path_constraints, 
                                            execution_info.descend_trajectory_.points.front().positions[2], 
                                            false)) 
    {
      //TODO: in the future, this should be hard stop, with an informative message back 
      //to the caller saying the constraints have failed
      ROS_WARN("Object place: move_arm to pre-place with constraints failed. Trying again without constraints.");
      use_constraints = false;
    }
  }
  //try to go to the pre-place pose without constraints
  if(!use_constraints)
  {
    ROS_DEBUG_NAMED("manipulation","Attempting move arm to pre-place without constraints");
    if ( !mechInterface().attemptMoveArmToGoal(place_goal.arm_name, 
                                               execution_info.descend_trajectory_.points.front().positions,
                                               place_goal.additional_collision_operations,
                                               place_goal.additional_link_padding) ) 
    {
      ROS_DEBUG_NAMED("manipulation","Object place: move_arm (without constraints) to pre-place reports failure");
      execution_info.result_ = Result(PlaceLocationResult::MOVE_ARM_FAILED, true);
      return;
    }
  }
  ROS_DEBUG_NAMED("manipulation"," Arm moved to pre-place");

  execution_info.result_ = placeApproach(place_goal, execution_info);
  if ( execution_info.result_.result_code != PlaceLocationResult::SUCCESS)
  {
    ROS_DEBUG_NAMED("manipulation"," Pre-place to place approach failed");
    execution_info.result_ = Result(PlaceLocationResult::PLACE_FAILED, false);
    return;
  }
  ROS_DEBUG_NAMED("manipulation"," Place trajectory done");

  mechInterface().detachAndAddBackObjectsAttachedToGripper(place_goal.arm_name, place_goal.collision_object_name);
  ROS_DEBUG_NAMED("manipulation"," Object detached");

  mechInterface().handPostureGraspAction(place_goal.arm_name, place_goal.grasp,
                                         object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE, -1);
  ROS_DEBUG_NAMED("manipulation"," Object released");

  execution_info.result_ = retreat(place_goal);
  if (execution_info.result_.result_code != PlaceLocationResult::SUCCESS)
  {
    execution_info.result_ = Result(PlaceLocationResult::RETREAT_FAILED, false);
    return;
  }
  execution_info.result_ = Result(PlaceLocationResult::SUCCESS, true);
}

/*! Right now we can only deal with a single orientation constraint. This function checks if the 
 constraints have additional stuff inside that we can not deal with.
*/
bool StandardPlacePerformer::constraintsUnderstandable(const arm_navigation_msgs::Constraints &constraints)
{
  if(constraints.position_constraints.empty() && constraints.orientation_constraints.empty() && 
     constraints.joint_constraints.empty() && constraints.visibility_constraints.empty())
    return true;
  if(constraints.orientation_constraints.size() == 1 && constraints.position_constraints.empty() 
     && constraints.joint_constraints.empty() && constraints.visibility_constraints.empty())
    return true;
  return false;
}

PlaceLocationResult 
ReactivePlacePerformer::placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
                                      const PlaceExecutionInfo &execution_info)
{
  //prepare the goal for reactive placing
  object_manipulation_msgs::ReactivePlaceGoal reactive_place_goal;  
  reactive_place_goal.arm_name = place_goal.arm_name;
  reactive_place_goal.collision_object_name = place_goal.collision_object_name;
  reactive_place_goal.collision_support_surface_name = place_goal.collision_support_surface_name;
  reactive_place_goal.trajectory = execution_info.descend_trajectory_;
  reactive_place_goal.final_place_pose = execution_info.gripper_place_pose_;

  //give the reactive place 1 minute to do its thing
  ros::Duration timeout = ros::Duration(60.0);
  ROS_DEBUG_NAMED("manipulation"," Calling the reactive place action");
  mechInterface().reactive_place_action_client_.client(place_goal.arm_name).sendGoal(reactive_place_goal);
  if ( !mechInterface().reactive_place_action_client_.client(place_goal.arm_name).waitForResult(timeout) )
  {
    ROS_ERROR("  Reactive place timed out");
    return Result(PlaceLocationResult::PLACE_FAILED, false);
  }
  object_manipulation_msgs::ReactivePlaceResult reactive_place_result = 
    *mechInterface().reactive_place_action_client_.client(place_goal.arm_name).getResult();
  if (reactive_place_result.manipulation_result.value != reactive_place_result.manipulation_result.SUCCESS)
  {
    ROS_ERROR("  Reactive place failed with error code %d", reactive_place_result.manipulation_result.value);
    return Result(PlaceLocationResult::PLACE_FAILED, false);
  }
  ROS_DEBUG_NAMED("manipulation","  Reactive place action succeeded");
  return Result(PlaceLocationResult::SUCCESS, true);
}

}
