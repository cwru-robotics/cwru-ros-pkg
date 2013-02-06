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

// Author(s): Kaijen Hsiao (code adapted from Gil's fast grasp tester)

#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"
#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/ik_tester_fast.h"

using arm_navigation_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

  IKTesterFast::IKTesterFast(planning_environment::CollisionModels* cm,
				   const std::string& plugin_name) 
  : redundancy_(2),
    cm_(cm),
    state_(NULL),
    kinematics_loader_("kinematics_base","kinematics::KinematicsBase")
{  
  ros::NodeHandle nh;

  vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("ik_tester_fast", 128);
  vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("ik_tester_fast_array", 128);

  const std::map<std::string, planning_models::KinematicModel::GroupConfig>& group_config_map = 
    getCollisionModels()->getKinematicModel()->getJointModelGroupConfigMap();

  for(std::map<std::string, planning_models::KinematicModel::GroupConfig>::const_iterator it = group_config_map.begin();
      it != group_config_map.end();
      it++) {
    kinematics::KinematicsBase* kinematics_solver = NULL;
    try
    {
      kinematics_solver = kinematics_loader_.createClassInstance(plugin_name);
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load. Error1: %s", ex.what());    //handle the class failing to load
      return;
    }
    if(!it->second.base_link_.empty() && !it->second.tip_link_.empty()) {
      ik_solver_map_[it->first] = new arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware(kinematics_solver,
                                                                                                          getCollisionModels(),
                                                                                                          it->first);
      ik_solver_map_[it->first]->setSearchDiscretization(.025);
    }
  }
}

IKTesterFast::~IKTesterFast()
{
  for(std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*>::iterator it = ik_solver_map_.begin();
      it != ik_solver_map_.end();
      it++) {
    delete it->second;
  }
}

planning_environment::CollisionModels* IKTesterFast::getCollisionModels() {
  if(cm_ == NULL) {
    return &mechInterface().getCollisionModels();
  } else {
    return cm_;
  }
}

planning_models::KinematicState* IKTesterFast::getPlanningSceneState() {
  if(state_ == NULL) {
    if(mechInterface().getPlanningSceneState() == NULL)
    {
      ROS_ERROR("Planning scene was NULL!  Did you forget to set it somewhere?  Getting new planning scene");
      const arm_navigation_msgs::OrderedCollisionOperations collision_operations;
      const std::vector<arm_navigation_msgs::LinkPadding> link_padding;
      mechInterface().getPlanningScene(collision_operations, link_padding);
    }				       
    return mechInterface().getPlanningSceneState();
  } 
  else {
    return state_;
  }
}


void IKTesterFast::getGroupLinks(const std::string& group_name,
                                    std::vector<std::string>& group_links)
{
  group_links.clear();
  const planning_models::KinematicModel::JointModelGroup* jmg = 
    getCollisionModels()->getKinematicModel()->getModelGroup(group_name);
  if(jmg == NULL) return;
  group_links = jmg->getGroupLinkNames();
}

void IKTesterFast::getGroupJoints(const std::string& group_name,
                                    std::vector<std::string>& group_joints)
{
  if(ik_solver_map_.find(group_name) == ik_solver_map_.end()) {
    ROS_ERROR_STREAM("No group for solver " << group_name);
    return;
  }
  group_joints = ik_solver_map_[group_name]->getJointNames();
}


void IKTesterFast::testIKSet(std::string arm_name, const std::vector<geometry_msgs::PoseStamped> &test_poses,
                             bool return_on_first_hit, std::vector<sensor_msgs::JointState> &solutions_arr,
                             std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> &error_codes)
{
  ros::WallTime start = ros::WallTime::now();

  error_codes.resize(test_poses.size());
  solutions_arr.resize(test_poses.size());
  std::map<int, int> outcome_count;

  state_ = NULL;
  planning_environment::CollisionModels* cm = getCollisionModels();
  planning_models::KinematicState* state = getPlanningSceneState();

  std::map<std::string, double> planning_scene_state_values;
  state->getKinematicStateValues(planning_scene_state_values);

  std::vector<std::string> end_effector_links, arm_links; 
  getGroupLinks(handDescription().gripperCollisionName(arm_name), end_effector_links);
  getGroupLinks(handDescription().armGroup(arm_name), arm_links);

  collision_space::EnvironmentModel::AllowedCollisionMatrix original_acm = cm->getCurrentAllowedCollisionMatrix();
  cm->disableCollisionsForNonUpdatedLinks(arm_name);
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_disable_acm = cm->getCurrentAllowedCollisionMatrix();
  collision_space::EnvironmentModel::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

  //turning off collisions for the arm associated with this end effector for group_all_arm_disable_acm
  for(unsigned int i = 0; i < arm_links.size(); i++) {
    group_all_arm_disable_acm.changeEntry(arm_links[i], true);
  }

  std_msgs::Header world_header;
  world_header.frame_id = cm->getWorldFrameId();

  //Check IK for each pose in turn
  for(unsigned int i = 0; i < test_poses.size(); i++) {

    //set kinematic state back to original
    state->setKinematicState(planning_scene_state_values);

    //first check to see if the gripper itself is in collision
    geometry_msgs::PoseStamped world_pose_stamped;
    if(!cm->convertPoseGivenWorldTransform(*state,
                                           world_header.frame_id,
                                           test_poses[i].header,
                                           test_poses[i].pose,
                                           world_pose_stamped)) {
      ROS_WARN_STREAM("Can't convert into frame " << world_header.frame_id);
      continue;
    }
    
    tf::Transform tf_pose;
    tf::poseMsgToTF(world_pose_stamped.pose, tf_pose);

    //only check the gripper for collisions, not the arm
    cm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);

    if(!state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(arm_name), tf_pose))
    {
      ROS_ERROR("ik_tester_fast: updateKinematicStateWithLinkAt failed!");
    }

    if(cm->isKinematicStateInCollision(*state)) {
      ROS_DEBUG("Kinematic state in collision!");
      outcome_count[arm_navigation_msgs::ArmNavigationErrorCodes::KINEMATICS_STATE_IN_COLLISION]++;
      error_codes[i].val = arm_navigation_msgs::ArmNavigationErrorCodes::KINEMATICS_STATE_IN_COLLISION;
      continue;
    } 
    
    //now check collision-aware ik for pose
    //go back to checking the entire arm
    cm->setAlteredAllowedCollisionMatrix(group_disable_acm);
    //cm->setAlteredAllowedCollisionMatrix(original_acm);
    state->setKinematicState(planning_scene_state_values);

    geometry_msgs::PoseStamped base_link_gripper_pose;
    cm->convertPoseGivenWorldTransform(*state,
                                       ik_solver_map_[arm_name]->getBaseName(),
                                       world_header,
                                       world_pose_stamped.pose,
                                       base_link_gripper_pose);

    arm_navigation_msgs::Constraints emp;
    sensor_msgs::JointState solution;
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    ROS_DEBUG_STREAM("x y z " << base_link_gripper_pose.pose.position.x << " " 
                     << base_link_gripper_pose.pose.position.y << " " 
                     << base_link_gripper_pose.pose.position.z);
    if(ik_solver_map_[arm_name]->findConstraintAwareSolution(base_link_gripper_pose.pose,
                                                              emp,
                                                              state,
                                                              solution,
                                                              error_code,
                                                              false)) {
      solutions_arr[i] = solution;
    }
    //didn't find a solution 
    else 
    {
      ROS_DEBUG("Pose out of reach or in collision");
    }

    //put the collision matrix back
    cm->setAlteredAllowedCollisionMatrix(original_acm);

    outcome_count[error_code.val]++;
    error_codes[i].val = error_code.val;
    if(return_on_first_hit && error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS) break;
  }

  ROS_INFO_STREAM("Took " << (ros::WallTime::now()-start).toSec());

  //print out how many of each outcome we found
  for(std::map<int, int>::iterator it = outcome_count.begin();
      it != outcome_count.end();
      it++) {
    ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
  }
}


} //namespace object_manipulator
