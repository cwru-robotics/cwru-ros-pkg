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

// Author(s): Kaijen Hsiao (based on Gil's grasp_tester_fast.h)

#ifndef _IK_TESTER_FAST_
#define _IK_TESTER_FAST_

#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
#include <pluginlib/class_loader.h>

namespace object_manipulator {

//! Checks a batch of IK queries at once

inline geometry_msgs::Vector3 doNegate(const geometry_msgs::Vector3& vec) {
  geometry_msgs::Vector3 v;
  v.x = - vec.x;
  v.y = - vec.y;
  v.z = - vec.z;
  return v;
}

class IKTesterFast
{
protected:
  
  std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*> ik_solver_map_;
  
  unsigned int redundancy_;
  
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

  planning_environment::CollisionModels* getCollisionModels();
  planning_models::KinematicState* getPlanningSceneState();

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* state_;

 public:

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;

  IKTesterFast(planning_environment::CollisionModels* cm = NULL,
		  const std::string& plugin_name="abby_irb_120_kinematics/IKFastKinematicsPlugin");

  ~IKTesterFast();

  void setPlanningSceneState(planning_models::KinematicState* state) {
    state_ = state;
  }

  void getGroupJoints(const std::string& group_name,
                      std::vector<std::string>& group_links);
  
  void getGroupLinks(const std::string& group_name,
                     std::vector<std::string>& group_links);

  void testIKSet(std::string arm_name, const std::vector<geometry_msgs::PoseStamped> &test_poses,
                 bool return_on_first_hit, std::vector<sensor_msgs::JointState> &solutions_arr,
                 std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> &error_codes);

};

} //namespace object_manipulator

#endif
