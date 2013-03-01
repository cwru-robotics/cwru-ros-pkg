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

#ifndef _APPROACH_LIFT_GRASP_
#define _APPROACH_LIFT_GRASP_

#include <ros/ros.h>

#include <object_manipulation_msgs/GraspResult.h>
#include <object_manipulation_msgs/PickupGoal.h>

#include <trajectory_msgs/JointTrajectory.h>

#include "object_manipulator/tools/grasp_marker_publisher.h"

namespace object_manipulator {

//! Encapsulates the result of feasibility testing and the information needed for eexcuting
//! a grasp, assuming an approach-grasp-lift method.
struct GraspExecutionInfo {
  trajectory_msgs::JointTrajectory approach_trajectory_; 
  trajectory_msgs::JointTrajectory lift_trajectory_; 
  object_manipulation_msgs::GraspResult result_;
  int marker_id_;
};

// ---------------------------- Definitions ---------------------------------

//! Tests grasps for feasibility in the current environment, and generates info
//! needed for execution
class GraspTester {
protected:
  //! Tests a single grasp
  virtual void testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                         const object_manipulation_msgs::Grasp &grasp,
                         GraspExecutionInfo &execution_info) = 0;  

  //! The marker publisher that will be used; NULL is marker publishing is disabled
  GraspMarkerPublisher *marker_publisher_;

  //! Function used to provide feedback on which grasp is being tested
  //! Might find a more elegant mechanism in the future
  boost::function<void(size_t)> feedback_function_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
public:
  GraspTester() : marker_publisher_(NULL) {}

  //! Tests a set of grasps and provides their execution info
  virtual void testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,
                          std::vector<GraspExecutionInfo> &execution_info,
                          bool return_on_first_hit);

  //! Sets the marker publisher to be used
  void setMarkerPublisher(GraspMarkerPublisher *pub){marker_publisher_ = pub;}

  //! Sets the feedback function
  void setFeedbackFunction(boost::function<void(size_t)> f){feedback_function_ = f;}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Helper function for convenience
  object_manipulation_msgs::GraspResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::GraspResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
};

//! Attempts to physically perform grasps, given the test results and information generated
//! by a grasp tester.
class GraspPerformer {
protected:
  //! Attempts to perform a single grasp
  virtual void performGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                            const object_manipulation_msgs::Grasp &grasp,
                            GraspExecutionInfo &execution_info) = 0;

  //! The marker publisher that will be used; NULL is marker publishing is disabled
  GraspMarkerPublisher *marker_publisher_;

  //! Function used to provide feedback on which grasp is being tested
  //! Might find a more elegant mechanism in the future
  boost::function<void(size_t)> feedback_function_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
public:
  GraspPerformer() : marker_publisher_(NULL) {}

  //! Attempts to perform a set of grasps
  virtual void performGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                             const std::vector<object_manipulation_msgs::Grasp> &grasps,
                             std::vector<GraspExecutionInfo> &execution_info);

  //! Sets the marker publisher to be used
  void setMarkerPublisher(GraspMarkerPublisher *pub){marker_publisher_ = pub;}

  //! Sets the feedback function
  void setFeedbackFunction(boost::function<void(size_t)> f){feedback_function_ = f;}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Helper function for convenience
  object_manipulation_msgs::GraspResult Result(int result_code, bool continuation)
  {
    object_manipulation_msgs::GraspResult result;
    result.result_code = result_code;
    result.continuation_possible = continuation;
    return result;
  }
};

// ------------------------------ Grasp Testers ----------------------------------

//! Standard grasp tester, attempts to compute Interpolated IK trajectories for both approach
//! and lift.
class GraspTesterWithApproach : public GraspTester
{
protected:
  virtual void testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                         const object_manipulation_msgs::Grasp &grasp,
                         GraspExecutionInfo &execution_info);  

  //! Calls the interpolated IK service to find a path from the grasp to lift the object
  object_manipulation_msgs::GraspResult 
  getInterpolatedIKForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                           const object_manipulation_msgs::Grasp &grasp,
                           const std::vector<double> &grasp_joint_angles,
                           GraspExecutionInfo &execution_info);
  
  //! Collision operations to be used when planning the lift motion
  virtual arm_navigation_msgs::OrderedCollisionOperations 
  collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                             const object_manipulation_msgs::Grasp &grasp);
  
  //! Dynamic link padding to be used when planning the lift motion
  virtual std::vector<arm_navigation_msgs::LinkPadding> 
  linkPaddingForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                     const object_manipulation_msgs::Grasp &grasp);  

  //! Computes an interpolated IK trajectory between pre_grasp and grasp and checks if it has enough points
  object_manipulation_msgs::GraspResult 
  getInterpolatedIKForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                            const object_manipulation_msgs::Grasp &grasp,
                            GraspExecutionInfo &execution_info);
  
  //! Collision operations to be used when planning the grasp motion
  virtual arm_navigation_msgs::OrderedCollisionOperations 
  collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                              const object_manipulation_msgs::Grasp &grasp);
  
  //! Dynamic link padding to be used for grasp operation
  virtual std::vector<arm_navigation_msgs::LinkPadding> 
  linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                      const object_manipulation_msgs::Grasp &grasp);

  //! Epsilon margin of error used when checking trajectory lengths
  static float EPS;
};

//! Does not care about collisions when planning approach and lift
class UnsafeGraspTester : public GraspTesterWithApproach
{
protected:
  //! Disables all collisions
  virtual arm_navigation_msgs::OrderedCollisionOperations 
  collisionOperationsForLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                             const object_manipulation_msgs::Grasp &grasp);

  //! Disables all collisions
  virtual arm_navigation_msgs::OrderedCollisionOperations 
  collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                              const object_manipulation_msgs::Grasp &grasp);
};

// ---------------------------- Grasp Performers ---------------------------------

//! Regular grasp performer: moveArm to beginning of approach, execute approach, grasp, execute lift
class StandardGraspPerformer : public GraspPerformer {
protected:
  virtual void performGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                            const object_manipulation_msgs::Grasp &grasp,
                            GraspExecutionInfo &execution_info);

  virtual object_manipulation_msgs::GraspResult 
  approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                   const object_manipulation_msgs::Grasp &grasp,
                   GraspExecutionInfo &execution_info);
  
  virtual object_manipulation_msgs::GraspResult 
  lift(const object_manipulation_msgs::PickupGoal &pickup_goal,
       const object_manipulation_msgs::Grasp &grasp,
       GraspExecutionInfo &execution_info);

  virtual object_manipulation_msgs::GraspResult 
  retreat(const object_manipulation_msgs::PickupGoal &pickup_goal,
          const object_manipulation_msgs::Grasp &grasp,
          GraspExecutionInfo &execution_info); 
};

//! Uses reactive grasping instead of executing approach trajectory, then computes a new lift
//! trajectory from wherever reactive grasping ends up.
class ReactiveGraspPerformer : public StandardGraspPerformer {
protected:

  //! Open loop lift that just executes a pre-set trajectory
  virtual object_manipulation_msgs::GraspResult 
  nonReactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
                  const object_manipulation_msgs::Grasp &grasp,
                  GraspExecutionInfo &execution_info);
  
  //! Lifting based on fingertip forces
  virtual object_manipulation_msgs::GraspResult 
  reactiveLift(const object_manipulation_msgs::PickupGoal &pickup_goal,
               const object_manipulation_msgs::Grasp &grasp,
               GraspExecutionInfo &execution_info);  
  
  virtual object_manipulation_msgs::GraspResult 
  lift(const object_manipulation_msgs::PickupGoal &pickup_goal,
       const object_manipulation_msgs::Grasp &grasp,
       GraspExecutionInfo &execution_info);  
  
  virtual object_manipulation_msgs::GraspResult 
  approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                   const object_manipulation_msgs::Grasp &grasp,
                   GraspExecutionInfo &execution_info);  
};


//! Uses open-loop Cartesian controllers for all movement
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
class UnsafeGraspPerformer : public ReactiveGraspPerformer {
protected:
  virtual object_manipulation_msgs::GraspResult 
  approachAndGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                   const object_manipulation_msgs::Grasp &grasp,
                   GraspExecutionInfo &execution_info);  
};

}

#endif
