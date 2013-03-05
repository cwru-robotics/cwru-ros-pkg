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

#ifndef _UNSAFE_GRASP_EXECUTOR_H_
#define _UNSAFE_GRASP_EXECUTOR_H_

#include "object_manipulator/grasp_execution/reactive_grasp_executor.h"

namespace object_manipulator {

//! Uses Cartesian movements to get to pre-grasp and grasp
/*! Initial check consists of non-collision-aware IK for the grasp and
pre-grasp.

Execution consists of using Cartesian controllers to move directly
to the pre-grasp, ignoring any possible collisions, then opening 
the hand to the desired pre-grasp configuration, then moving from 
pre-grasp to grasp again with Cartesian controllers (reactively 
if desired).

This is intended for crowded situations where the hand is already close 
to the pre-grasp and we want to just move directly there, shove things 
out of the way, and grasp.
*/

class UnsafeGraspExecutor : public ReactiveGraspExecutor
{
 protected:

  //! Uses Cartesian movement to get to pre-grasp, then either reactive 
  // or non-reactive grasping to grasp
  virtual object_manipulation_msgs::GraspResult 
    executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp);

  //! Disables collisions when planning the grasp motion
  virtual arm_navigation_msgs::OrderedCollisionOperations 
    collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal);

  //! Collision operations to be used when planning the lift motion
  arm_navigation_msgs::OrderedCollisionOperations 
    collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal);

 public:
  //! Only calls super constructor
  UnsafeGraspExecutor(GraspMarkerPublisher *pub) : ReactiveGraspExecutor(pub) {}

};

} //namespace object_manipulator

#endif
