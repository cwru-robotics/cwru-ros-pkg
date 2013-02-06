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

#ifndef _GRASP_EXECUTOR_H_
#define _GRASP_EXECUTOR_H_

#include <ros/ros.h>

#include "object_manipulation_msgs/GraspResult.h"
#include "object_manipulation_msgs/PickupGoal.h"

#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/grasp_marker_publisher.h"

namespace object_manipulator {

//! Base class, interface and common functionality for grasp execution. 
/*! Defines the interface for a grasp executor:

 */
class GraspExecutor
{
 public:
  object_manipulation_msgs::GraspResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::GraspResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
  
 protected:

  //! The marker publisher that will be used; NULL is marker publishing is disabled
  GraspMarkerPublisher *marker_publisher_;

  //! The marker id of this grasp, retrieved from the publisher
  unsigned int marker_id_;

  //! The result of interpolated IK from grasp to lift
  trajectory_msgs::JointTrajectory interpolated_lift_trajectory_; 

  //! Calls the interpolated IK service to find a path from the grasp to lift the object
  object_manipulation_msgs::GraspResult 
    getInterpolatedIKForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
			     const object_manipulation_msgs::Grasp &grasp,
			     const std::vector<double> &grasp_joint_angles,
			     trajectory_msgs::JointTrajectory &lift_trajectory);

  //! Collision operations to be used when planning the lift motion
  arm_navigation_msgs::OrderedCollisionOperations 
    collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal);

  //! Dynamic link padding to be used when planning the lift motion
  std::vector<arm_navigation_msgs::LinkPadding> 
    linkPaddingForLift(const object_manipulation_msgs::PickupGoal &pickup_goal);

  //! Helper function for setting padding on fingertips
  std::vector<arm_navigation_msgs::LinkPadding> 
    fingertipPadding(const object_manipulation_msgs::PickupGoal &pickup_goal, double pad);

  //! Performs an initial check to see if the grasp is valid and generates information needed to execute it
  virtual object_manipulation_msgs::GraspResult 
    prepareGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp) = 0;

  //! Implementation of the actual execution stage of the grasp
  /*! Can use information generated during prepareGrasp */
  virtual object_manipulation_msgs::GraspResult
    executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp) = 0;

 public:

  //! Also sets the marker publisher that is to be used; pass NULL to disable marker publishing
  GraspExecutor(GraspMarkerPublisher *pub) : marker_publisher_(pub)
  {}
  virtual ~GraspExecutor() {}
  
  //! Checks if the grasp is feasible, then executes it 
  /* Returns SUCCESS if the grasp was successfully executed, UNFEASIBLE if it fails without
     affecting the environment (generally during move_arm stages) or FAILED if it fails and might
     have affected the object (generally during the final grasp stage).
  
   Calls prepareGrasp(), and if prepareGrasp() returns true, it calls executeGrasp() and returns its result.
  */  
  object_manipulation_msgs::GraspResult 
    checkAndExecuteGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
			 const object_manipulation_msgs::Grasp &grasp);
  
  //! Called if the grasp fails to retreat the gripper
  /*! By default, a grasp executor does not know how to do this.
    Generally returns UNFEASIBLE if no movement has been performed at all, SUCCESS if a full retreat
    was achieved and FAILED is some movement was performed but it fell short of a full retreat. */
  virtual object_manipulation_msgs::GraspResult 
  retreat(const object_manipulation_msgs::PickupGoal &pickup_goal, const object_manipulation_msgs::Grasp &grasp)
  {
    ROS_WARN("This grasp executor has no retreat capability");
    return Result(object_manipulation_msgs::GraspResult::RETREAT_FAILED, false);
  }

  //! Lifts the object after grasping 
  /*! Generally returns SUCCESS if full lift was achieved, UNFEASIBLE if no movement was performed and 
    FAILED if some movement was achieved but it fell short of a full lift. 
    
    Will often use information that was pre-generated during grasp execution, so this should always
    be called after executeGrasp() has succeeded.*/
  virtual object_manipulation_msgs::GraspResult lift(const object_manipulation_msgs::PickupGoal &pickup_goal);
};

} //namespace grasp_execution

#endif
