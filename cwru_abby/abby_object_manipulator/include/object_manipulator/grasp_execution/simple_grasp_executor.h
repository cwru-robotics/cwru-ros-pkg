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

#ifndef _SIMPLE_GRASP_EXECUTOR_H_
#define _SIMPLE_GRASP_EXECUTOR_H_

#include "object_manipulator/grasp_execution/grasp_executor.h"

namespace object_manipulator {

//! Simplest grasp executor, using only final grasp information
/*! The initial check consists of calling IK on the final grasp to see if
  a valid IK solution exists, then cheking interpolated IK to see if the 
  object can be lifted. Execution then consists of passing along
  the IK solution to move_arm, followed by the lift trajectory.
 */
class SimpleGraspExecutor : public GraspExecutor
{
 private:
  //! The inverse kinematics solution for the final grasp
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response_;
  
  //! Calls IK on the final grasp pose
  virtual object_manipulation_msgs::GraspResult 
    prepareGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp);
  
  //! Sends the IK result of prepareGrasp() to move_arm, then executes lift trajectory
  virtual object_manipulation_msgs::GraspResult 
    executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
		 const object_manipulation_msgs::Grasp &grasp);
 public:
  //! Also adds a marker at the final grasp
 SimpleGraspExecutor(GraspMarkerPublisher *pub) : GraspExecutor(pub) {}
  
};

} //namespace grasp execution

#endif
