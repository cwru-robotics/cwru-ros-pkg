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

#ifndef _PLACE_EXECUTOR_H_
#define _PLACE_EXECUTOR_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <tf/transform_listener.h>

#include <object_manipulation_msgs/PlaceGoal.h>
#include <object_manipulation_msgs/PlaceLocationResult.h>

#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/grasp_marker_publisher.h"

namespace object_manipulator {

//! Functionality for placing a grasped object at a given location
/*! As a mirror image of the classes that offer grasping functionality, placing implies:
  - a pre-place position (some cm above the final placing pose)
  - an interpolated trajectory from pre-place to place
  - a gripper retreat after placing, done by interpolating "back" along gripper approach direction

  This class will first compute that both place and retreat interpolated trajectories
  are possible, then proceed to execute the place trajectory.
*/
class PlaceExecutor
{
public:
  object_manipulation_msgs::PlaceLocationResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::PlaceLocationResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
protected:
  //! Publisher for markers, or NULL if markers are not to be published
  GraspMarkerPublisher *marker_publisher_;

  //! Id of published marker, if any
  int marker_id_;

  //! The interpolated trajectory used to place object
  trajectory_msgs::JointTrajectory place_trajectory_;

  //! The interpolated trajectory used to retreat after place
  trajectory_msgs::JointTrajectory retreat_trajectory_;

  //! Transform listener 
  tf::TransformListener listener_;

  //! Computes the gripper pose for a desired object place location
  geometry_msgs::PoseStamped computeGripperPose(geometry_msgs::PoseStamped place_location, 
						geometry_msgs::Pose grasp_pose,
						std::string frame_id);

  //! Computes interpolated trajectories for both place and retreat
  object_manipulation_msgs::PlaceLocationResult 
    prepareInterpolatedTrajectories(const object_manipulation_msgs::PlaceGoal &place_goal,
				    const geometry_msgs::PoseStamped &place_location);

  //! Moves the gripper "back"; called after placing is done
  object_manipulation_msgs::PlaceLocationResult
    retreat(const object_manipulation_msgs::PlaceGoal &place_goal);
  
  //! Checks if the constraints are of the type that we can handle for the moment
  bool constraintsUnderstandable(const arm_navigation_msgs::Constraints &constraints);

  //! Goes from the pre-place location to the place location
  virtual object_manipulation_msgs::PlaceLocationResult
    placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
		  const geometry_msgs::PoseStamped &place_location);

public:
  PlaceExecutor(GraspMarkerPublisher *marker_publisher) : marker_publisher_(marker_publisher), marker_id_(-1)
  {}
  
  //! Places a grasped object at a specified location
  object_manipulation_msgs::PlaceLocationResult 
    place(const object_manipulation_msgs::PlaceGoal &place_goal, 
	  const geometry_msgs::PoseStamped &place_location);
};

//! Uses a reactive version of the move from pre-place to place
/*! Inherits all functionality from the PlaceExecutor except for the move from 
  pre-place to place, for which it calls a ReactivePlace action.
*/
class ReactivePlaceExecutor : public PlaceExecutor
{
protected:
  //! Calls the ReactivePlace action
  virtual object_manipulation_msgs::PlaceLocationResult 
    placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
		  const geometry_msgs::PoseStamped &place_location);
public:
  //! Empty stub
  ReactivePlaceExecutor(GraspMarkerPublisher *marker_publisher) : PlaceExecutor(marker_publisher)
  {}
};

} //namespace object_manipulator

#endif
