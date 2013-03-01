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

#ifndef _REACTIVE_GRASP_EXECUTOR_H_
#define _REACTIVE_GRASP_EXECUTOR_H_

#include "object_manipulator/grasp_execution/grasp_executor_with_approach.h"

namespace object_manipulator {

class ReactiveGraspExecutor : public GraspExecutorWithApproach
{
 protected:

  //! Uses move_arm to get to pre-grasp, then reactive grasping to grasp
  virtual object_manipulation_msgs::GraspResult 
    executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp);

  //! Open loop lift that just executes a pre-set trajectory
  virtual object_manipulation_msgs::GraspResult 
    nonReactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal);

  //! Lifting based on fingertip forces
  virtual object_manipulation_msgs::GraspResult 
    reactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal);  

 public:
  //! Only calls super constructor
  ReactiveGraspExecutor(GraspMarkerPublisher *pub) : GraspExecutorWithApproach(pub) {}

  //! Lifts the object starting from current gripper pose
  virtual object_manipulation_msgs::GraspResult 
    lift(const object_manipulation_msgs::PickupGoal &pickup_goal);  
};

} //namespace object_manipulator

#endif
