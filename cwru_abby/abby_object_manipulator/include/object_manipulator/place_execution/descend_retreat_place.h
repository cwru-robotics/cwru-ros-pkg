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

#ifndef _DESCEND_RETREAT_PLACE_
#define _DESCEND_RETREAT_PLACE_

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <object_manipulation_msgs/PlaceLocationResult.h>
#include <object_manipulation_msgs/PlaceGoal.h>

#include <trajectory_msgs/JointTrajectory.h>

#include "object_manipulator/tools/grasp_marker_publisher.h"

namespace object_manipulator {

//! Encapsulates the result of feasibility testing and the information needed for eexcuting
//! a place, assuming a descend-open-retreat method.
struct PlaceExecutionInfo {
  trajectory_msgs::JointTrajectory descend_trajectory_; 
  trajectory_msgs::JointTrajectory retreat_trajectory_; 
  geometry_msgs::PoseStamped gripper_place_pose_;
  object_manipulation_msgs::PlaceLocationResult result_;
  int marker_id_;
};

// ---------------------------- Definitions ---------------------------------

//! Tests place locations for feasibility in the current environment, and generates info
//! needed for execution
class PlaceTester {
protected:
  //! Transform listener 
  tf::TransformListener listener_;

  //! Computes the gripper pose for a desired object place location
  geometry_msgs::PoseStamped computeGripperPose(geometry_msgs::PoseStamped place_location, 
						geometry_msgs::Pose grasp_pose,
						std::string frame_id);

  //! Tests a single place location
  virtual void testPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                         const geometry_msgs::PoseStamped &place_location,
                         PlaceExecutionInfo &execution_info) = 0;  

  //! The marker publisher that will be used; NULL is marker publishing is disabled
  GraspMarkerPublisher *marker_publisher_;

  //! Function used to provide feedback on which place is being tested
  //! Might find a more elegant mechanism in the future
  boost::function<void(size_t)> feedback_function_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;

public:
  PlaceTester() : marker_publisher_(NULL) {}

  //! Tests a set of place locations and provides their execution info
  virtual void testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
                          const std::vector<geometry_msgs::PoseStamped> &place_locations,
                          std::vector<PlaceExecutionInfo> &execution_info,
                          bool return_on_first_hit);

  //! Sets the marker publisher to be used
  void setMarkerPublisher(GraspMarkerPublisher *pub){marker_publisher_ = pub;}

  //! Sets the feedback function
  void setFeedbackFunction(boost::function<void(size_t)> f){feedback_function_ = f;}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Helper function for convenience
  object_manipulation_msgs::PlaceLocationResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::PlaceLocationResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
};

//! Attempts to physically perform a place task, given the test results and information generated
//! by a place tester.
class PlacePerformer {
protected:
  //! Attempts to perform a single place location
  virtual void performPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                            const geometry_msgs::PoseStamped &place_location,
                            PlaceExecutionInfo &execution_info) = 0;

  //! The marker publisher that will be used; NULL is marker publishing is disabled
  GraspMarkerPublisher *marker_publisher_;

  //! Function used to provide feedback on which grasp is being tested
  //! Might find a more elegant mechanism in the future
  boost::function<void(size_t)> feedback_function_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;

public:
  PlacePerformer() : marker_publisher_(NULL) {}

  //! Attempts to perform a set of place locations
  virtual void performPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
                             const std::vector<geometry_msgs::PoseStamped> &place_locations,
                             std::vector<PlaceExecutionInfo> &execution_info);

  //! Sets the marker publisher to be used
  void setMarkerPublisher(GraspMarkerPublisher *pub){marker_publisher_ = pub;}

  //! Sets the feedback function
  void setFeedbackFunction(boost::function<void(size_t)> f){feedback_function_ = f;}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Helper function for convenience
  object_manipulation_msgs::PlaceLocationResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::PlaceLocationResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
};

// ---------------------------- Testers ---------------------------------

class StandardPlaceTester : public PlaceTester
{
protected:
  //! Tests both descent and retreat using interpolated IK
  virtual void testPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                         const geometry_msgs::PoseStamped &place_location,
                         PlaceExecutionInfo &execution_info);  

  //! Numerical error margin when checking if trajectories are long enough
  static float EPS;
};

// ---------------------------- Performers ---------------------------------

class StandardPlacePerformer : public PlacePerformer
{
  //! Moves the gripper "back"; called after placing is done
  object_manipulation_msgs::PlaceLocationResult
    retreat(const object_manipulation_msgs::PlaceGoal &place_goal);
  
  //! Checks if the constraints are of the type that we can handle for the moment
  bool constraintsUnderstandable(const arm_navigation_msgs::Constraints &constraints);

  //! Goes from the pre-place location to the place location
  virtual object_manipulation_msgs::PlaceLocationResult
    placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
                  const PlaceExecutionInfo &execution_info);

  //! Attempts to perform a single place location
  virtual void performPlace(const object_manipulation_msgs::PlaceGoal &place_goal,
                            const geometry_msgs::PoseStamped &place_location,
                            PlaceExecutionInfo &execution_info);
};

//! Uses a reactive version of the move from pre-place to place
/*! Inherits all functionality from the StandardPlacePerformer except for the move from 
  pre-place to place, for which it calls a ReactivePlace action.
*/
class ReactivePlacePerformer : public StandardPlacePerformer
{
protected:
  //! Calls the ReactivePlace action
  virtual object_manipulation_msgs::PlaceLocationResult 
    placeApproach(const object_manipulation_msgs::PlaceGoal &place_goal,
                  const PlaceExecutionInfo &execution_info);
};


}


#endif
