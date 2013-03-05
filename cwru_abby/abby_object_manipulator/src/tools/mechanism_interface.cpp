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

#include "object_manipulator/tools/mechanism_interface.h"
#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/exceptions.h"

//#define PROF_ENABLED
//#include <profiling/profiling.h>
//PROF_DECLARE(INTERPOLATED_IK);
//PROF_DECLARE(SET_PLANNING_SCENE);

namespace object_manipulator {

static const std::string IK_SERVICE_SUFFIX = "/constraint_aware_ik";
static const std::string FK_SERVICE_SUFFIX = "/get_fk";
static const std::string INTERPOLATED_IK_SERVICE_SUFFIX = "/interpolated_ik";
static const std::string INTERPOLATED_IK_SET_PARAMS_SERVICE_SUFFIX = "/interpolated_ik_set_params";
static const std::string IK_QUERY_SERVICE_SUFFIX = "/get_ik_solver_info";
static const std::string GRASP_STATUS_SUFFIX = "/grasp_status";

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "environment_server/set_planning_scene_diff";
static const std::string CHECK_STATE_VALIDITY_NAME = "planning_scene_validity_server/get_state_validity";
static const std::string GET_ROBOT_STATE_NAME = "environment_server/get_robot_state";
static const std::string NORMALIZE_SERVICE_NAME = "trajectory_filter_unnormalizer/filter_trajectory";
static const std::string RESET_COLLISION_MAP_SERVICE_NAME = "collider_node/reset";

static const std::string REACTIVE_GRASP_ACTION_SUFFIX = "/reactive_grasp";
static const std::string REACTIVE_LIFT_ACTION_SUFFIX = "/reactive_lift";
static const std::string REACTIVE_PLACE_ACTION_SUFFIX = "/reactive_place";
static const std::string MOVE_ARM_ACTION_SUFFIX = "/move_arm";
static const std::string TRAJECTORY_ACTION_SUFFIX = "/joint_trajectory";
static const std::string HAND_POSTURE_ACTION_SUFFIX = "/hand_posture_execution";

static const std::string SWITCH_CONTROLLER_SERVICE_NAME = "/switch_controller";
static const std::string LIST_CONTROLLERS_SERVICE_NAME = "/list_controllers";
static const std::string CARTESIAN_COMMAND_SUFFIX = "/cart/command_pose";
static const std::string CARTESIAN_POSTURE_SUFFIX = "/cart/command_posture";
//static const std::string CARTESIAN_GAINS_SUFFIX = "/cart/gains";

static const std::string MOVE_ARM_PLANNER_ID = "SBLkConfig1";
static const std::string MOVE_ARM_PLANNER_SERVICE_NAME = "ompl_planning/plan_kinematic_path";
static const std::string MOVE_ARM_CONSTRAINED_PLANNER_SERVICE_NAME = "ompl_planning/plan_kinematic_path";

static const std::string ATTACHED_COLLISION_TOPIC="attached_collision_object";

static const std::string POINT_HEAD_ACTION_TOPIC = "/head_traj_controller/point_head_action";

static const double OBJECT_POSITION_TOLERANCE_X = 0.02;
static const double OBJECT_POSITION_TOLERANCE_Y = 0.02;
static const double OBJECT_POSITION_TOLERANCE_Z = 0.02;

MechanismInterface::MechanismInterface() : 
  root_nh_(""),priv_nh_("~"),
  cm_("robot_description"),
  planning_scene_state_(NULL),
  planning_scene_cache_empty_(true),
  cache_planning_scene_(false),
  //------------------- multi arm service clients -----------------------
  ik_query_client_("", IK_QUERY_SERVICE_SUFFIX, true),
  ik_service_client_("", IK_SERVICE_SUFFIX, true),
  fk_service_client_("",FK_SERVICE_SUFFIX,true),
  interpolated_ik_service_client_("", INTERPOLATED_IK_SERVICE_SUFFIX, true),
  interpolated_ik_set_params_client_("", INTERPOLATED_IK_SET_PARAMS_SERVICE_SUFFIX, true),
  grasp_status_client_("", GRASP_STATUS_SUFFIX, true),
  //------------------- simple service clients -----------------------
  check_state_validity_client_(CHECK_STATE_VALIDITY_NAME),
  joint_trajectory_normalizer_service_(NORMALIZE_SERVICE_NAME),
  switch_controller_service_(SWITCH_CONTROLLER_SERVICE_NAME),
  list_controllers_service_(LIST_CONTROLLERS_SERVICE_NAME),
  get_robot_state_client_(GET_ROBOT_STATE_NAME),
  set_planning_scene_diff_service_(SET_PLANNING_SCENE_DIFF_NAME),
  reset_collision_map_service_(RESET_COLLISION_MAP_SERVICE_NAME),
  //-------------------- multi arm action clients -----------------------
  reactive_grasp_action_client_("", REACTIVE_GRASP_ACTION_SUFFIX, true, true),
  reactive_lift_action_client_("", REACTIVE_LIFT_ACTION_SUFFIX, true, true),
  reactive_place_action_client_("", REACTIVE_PLACE_ACTION_SUFFIX, true, true),
  move_arm_action_client_("", MOVE_ARM_ACTION_SUFFIX, true, true),
  traj_action_client_("", TRAJECTORY_ACTION_SUFFIX, true, true),
  hand_posture_client_("", HAND_POSTURE_ACTION_SUFFIX, true, true),
  //-------------------- head action client -----------------------
  point_head_action_client_(POINT_HEAD_ACTION_TOPIC, true),
  //-------------------- multi arm topic publishers ---------------------
  cartesian_pub_("", CARTESIAN_COMMAND_SUFFIX, true),
  cartesian_posture_pub_("", CARTESIAN_POSTURE_SUFFIX, true)
  //cartesian_gains_pub_("", CARTESIAN_GAINS_SUFFIX, true)
{
  //attached object publishing topic
  attached_object_pub_ = root_nh_.advertise<arm_navigation_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_TOPIC, 10);

  //JointStates topic for current arm angles
  priv_nh_.param<std::string>("joint_states_topic", joint_states_topic_, "joint_states");

}

/*! For now, just calls the IK Info service each time. In the future, we might do some
 caching in here.
*/
std::vector<std::string> MechanismInterface::getJointNames(std::string arm_name)
{
  kinematics_msgs::GetKinematicSolverInfo::Request query_request;
  kinematics_msgs::GetKinematicSolverInfo::Response query_response;  
  if ( !ik_query_client_.client(arm_name).call(query_request, query_response) ) 
  {
    ROS_ERROR("Failed to call ik information query");
    throw MechanismException("Failed to call ik information query");
  }
  return query_response.kinematic_solver_info.joint_names;
}

void MechanismInterface::getRobotState(arm_navigation_msgs::RobotState& robot_state)
{
  arm_navigation_msgs::GetRobotState::Request req;
  arm_navigation_msgs::GetRobotState::Response res;  
  if(!get_robot_state_client_.client().call(req,res)) 
  {
    ROS_ERROR("Mechanism interface: can't get current robot state");
    throw MechanismException("Mechanism interface: can't get current robot state");
  }
  robot_state = res.robot_state;
}

bool compareOrderedCollisionOperations(const arm_navigation_msgs::OrderedCollisionOperations& c1,
                                       const arm_navigation_msgs::OrderedCollisionOperations& c2)
{
  if (c1.collision_operations.size() != c2.collision_operations.size()) return false;
  for (size_t i=0; i<c1.collision_operations.size(); i++)
  {
    if (c1.collision_operations[i].object1 != c2.collision_operations[i].object1 ) return false;
    if (c1.collision_operations[i].object2 != c2.collision_operations[i].object2 ) return false;
    if (c1.collision_operations[i].penetration_distance != 
        c2.collision_operations[i].penetration_distance ) return false;
    if (c1.collision_operations[i].operation != c2.collision_operations[i].operation ) return false;
  }
  return true;
}

bool compareLinkPadding(const std::vector<arm_navigation_msgs::LinkPadding> &l1,
                        const std::vector<arm_navigation_msgs::LinkPadding> &l2)
{
  if (l1.size() != l2.size()) return false;
  for (size_t i=0; i<l1.size(); i++)
  {
    if (l1[i].link_name != l2[i].link_name) return false;
    if (l1[i].padding != l2[i].padding) return false;
  }
  return true;
}

bool MechanismInterface::cachePlanningScene(const arm_navigation_msgs::OrderedCollisionOperations& collision_operations,
                                            const std::vector<arm_navigation_msgs::LinkPadding> &link_padding)
{
  static int cache_hits = 0;
  static int total_hits = 0;
  total_hits++;
  if (!cache_planning_scene_) 
  {
    ROS_DEBUG_NAMED("manipulation","Planning scene caching disabled");
    collision_operations_cache_ = collision_operations;
    link_padding_cache_ = link_padding;
    return false;
  }
  if (planning_scene_cache_empty_)
  {
    ROS_DEBUG_NAMED("manipulation","Planning scene cache empty.");
    collision_operations_cache_ = collision_operations;
    link_padding_cache_ = link_padding;
    planning_scene_cache_empty_ = false;
    return false;
  }
  if (!compareOrderedCollisionOperations(collision_operations, collision_operations_cache_))
  {
    ROS_DEBUG_NAMED("manipulation","Planning scene cache miss - collisions (hits: %d/%d).", cache_hits, total_hits);
    collision_operations_cache_ = collision_operations;
    link_padding_cache_ = link_padding;
    return false;
  }
  if (!compareLinkPadding(link_padding, link_padding_cache_))
  {
    ROS_DEBUG_NAMED("manipulation","Planning scene cache miss - padding (hits: %d/%d).", cache_hits, total_hits);
    collision_operations_cache_ = collision_operations;
    link_padding_cache_ = link_padding;
    return false;
  }
  cache_hits++;
  ROS_DEBUG_NAMED("manipulation", "Planning scene cache hit (hits: %d/%d).", cache_hits, total_hits);
  return true;
}
  
void MechanismInterface::getPlanningScene(const arm_navigation_msgs::OrderedCollisionOperations& collision_operations,
                                          const std::vector<arm_navigation_msgs::LinkPadding> &link_padding)
{
  //if (cachePlanningScene(collision_operations, link_padding)) return;
  arm_navigation_msgs::SetPlanningSceneDiff::Request planning_scene_req;
  planning_scene_req.planning_scene_diff.link_padding = link_padding;
  planning_scene_req.operations = collision_operations;
  arm_navigation_msgs::SetPlanningSceneDiff::Response planning_scene_res;  
  //PROF_COUNT(SET_PLANNING_SCENE);
  //PROF_START_TIMER(SET_PLANNING_SCENE);
  //ROS_INFO("mechanism_interface: setting the planning scene diff");
  if(!set_planning_scene_diff_service_.client().call(planning_scene_req, planning_scene_res)) 
  {
    ROS_ERROR("Failed to set planning scene diff");
    throw MechanismException("Failed to set planning scene diff");
  }
  
  if(planning_scene_state_ != NULL) {
    cm_.revertPlanningScene(planning_scene_state_);
  }
  planning_scene_state_ = cm_.setPlanningScene(planning_scene_res.planning_scene);
  //PROF_STOP_TIMER(SET_PLANNING_SCENE);
}

trajectory_msgs::JointTrajectory MechanismInterface::assembleJointTrajectory(std::string arm_name, 
					   const std::vector< std::vector<double> > &positions, 
					   float time_per_segment)
{
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.header.frame_id = handDescription().robotFrame(arm_name);
  trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  trajectory.joint_names = getJointNames(arm_name);
  float current_time = 0;
  for (size_t i=0; i<positions.size(); i++)
  {
    current_time += time_per_segment;
    if (positions[i].size() != trajectory.joint_names.size())
    {
      ROS_ERROR("Mechanism interface: requested trajectory does not have enough joint values");
      throw MechanismException("Mechanism interface: requested trajectory does not have enough joint values");
    }
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions[i];
    point.time_from_start = ros::Duration(current_time);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

void MechanismInterface::attemptTrajectory(std::string arm_name,
                                           const std::vector< std::vector<double> > &positions,
                                           bool unnormalize,
                                           float time_per_segment)
{
  attemptTrajectory(arm_name, assembleJointTrajectory(arm_name, positions, time_per_segment), unnormalize);
}


void MechanismInterface::unnormalizeTrajectory(std::string arm_name, 
					   const trajectory_msgs::JointTrajectory &input_trajectory,
					   trajectory_msgs::JointTrajectory &normalized_trajectory)
{    
  arm_navigation_msgs::FilterJointTrajectory service_call;
  getRobotState(service_call.request.start_state);
  service_call.request.trajectory = input_trajectory;
  service_call.request.allowed_time = ros::Duration(2.0);
  if ( !joint_trajectory_normalizer_service_.client().call(service_call) )
  {
    ROS_ERROR("Mechanism interface: joint trajectory normalizer service call failed");
    throw MechanismException("joint trajectory normalizer service call failed");
  } 
  normalized_trajectory = service_call.response.trajectory;
}


void MechanismInterface::attemptTrajectory(std::string arm_name, 
					   const trajectory_msgs::JointTrajectory &trajectory, 
					   bool unnormalize)
{
  if (trajectory.points.empty()) 
  {
    ROS_ERROR("attemptTrajectory called with empty trajectory");
    throw MechanismException("attemptTrajectory called with empty trajectory");
  }

  //make sure joint controllers are running
  if(!checkController(jointControllerName(arm_name)))
     switchToJoint(arm_name);

  pr2_controllers_msgs::JointTrajectoryGoal goal;
  if (unnormalize)
  {
    unnormalizeTrajectory(arm_name, trajectory, goal.trajectory);
  }
  else
  {
    goal.trajectory = trajectory;
  }
  //force it to wait for client here, so that the duration below is not wasted
  traj_action_client_.client(arm_name);
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

  //wait 5 seconds more that the whole trajectory is supposed to take
  ros::Duration timeout = ros::Duration(1.0) + trajectory.points.back().time_from_start + ros::Duration(5.0);
  traj_action_client_.client(arm_name).sendGoal(goal);
  if ( !traj_action_client_.client(arm_name).waitForResult(timeout) ) 
  {
    ROS_ERROR("  Trajectory timed out");
    throw MechanismException("trajectory timed out");
  }
}

void MechanismInterface::setInterpolatedIKParams(std::string arm_name, int num_steps,
						 int collision_check_resolution, bool start_from_end)
{
  interpolated_ik_motion_planner::SetInterpolatedIKMotionPlanParams srv;
  srv.request.num_steps = num_steps;
  srv.request.consistent_angle = M_PI/6;
  srv.request.collision_check_resolution = collision_check_resolution;
  srv.request.steps_before_abort = 0;
  srv.request.pos_spacing = 0.01; //ignored if num_steps !=0
  srv.request.rot_spacing = 0.1;  //ignored if num_steps !=0
  srv.request.collision_aware = true;
  srv.request.start_from_end = start_from_end;
  if (!interpolated_ik_set_params_client_.client(arm_name).call(srv))
  {
    ROS_ERROR("Failed to set Interpolated IK server parameters");
    throw MechanismException("Failed to set Interpolated IK server parameters");
  }
}

bool MechanismInterface::moveArmToPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
                                       const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                       const std::vector<arm_navigation_msgs::LinkPadding> &link_padding)
{
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response;
  if(!getIKForPose(arm_name, desired_pose,ik_response, collision_operations, link_padding)) return false;
  if(!attemptMoveArmToGoal(arm_name, ik_response.solution.joint_state.position, collision_operations, link_padding))
    return false;
  return true;
}

bool MechanismInterface::getFK(std::string arm_name, 
			       std::vector<double> positions, 
			       geometry_msgs::PoseStamped &pose_stamped)
{
 // define the service messages
 kinematics_msgs::GetPositionFK::Request  fk_request;
 kinematics_msgs::GetPositionFK::Response fk_response;
 fk_request.header.frame_id = pose_stamped.header.frame_id;
 fk_request.header.stamp = pose_stamped.header.stamp;
 fk_request.fk_link_names.resize(1);
 fk_request.fk_link_names[0] = handDescription().gripperFrame(arm_name);
 fk_request.robot_state.joint_state.position = positions;
 fk_request.robot_state.joint_state.name = getJointNames(arm_name);
 if( !fk_service_client_.client(arm_name).call(fk_request,fk_response) ) 
   {
     ROS_ERROR("FK Service Call failed altogether");
     throw MechanismException("FK Service Call failed altogether");
   }
 if (fk_response.error_code.val != fk_response.error_code.SUCCESS)
   {
    ROS_ERROR("Get FK failed with error code %d", fk_response.error_code.val);
    return false;
   }
 pose_stamped = fk_response.pose_stamped[0];
 return true;
}

bool MechanismInterface::getIKForPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
                                      kinematics_msgs::GetConstraintAwarePositionIK::Response& ik_response,
                                      const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                      const std::vector<arm_navigation_msgs::LinkPadding> &link_padding)
{
  //prepare the planning scene
  getPlanningScene(collision_operations, link_padding);
  //call collision-aware ik
  kinematics_msgs::GetConstraintAwarePositionIK::Request ik_request;
  ik_request.ik_request.ik_link_name = handDescription().gripperFrame(arm_name);
  ik_request.ik_request.pose_stamped.pose = desired_pose.pose;
  ik_request.ik_request.pose_stamped.header.stamp = desired_pose.header.stamp;
  ik_request.ik_request.pose_stamped.header.frame_id = desired_pose.header.frame_id;
  ik_request.ik_request.ik_seed_state.joint_state.name = getJointNames(arm_name);
  ik_request.ik_request.ik_seed_state.joint_state.position.resize(7, 0.0);
  ik_request.timeout = ros::Duration(2.0);
  if( !ik_service_client_.client(arm_name).call(ik_request,ik_response) ) 
  {
    ROS_ERROR("IK Service Call failed altogether");
    throw MechanismException("IK Service Call failed altogether");
  }
  if (ik_response.error_code.val != ik_response.error_code.SUCCESS)
  {
    ROS_ERROR("Get IK failed with error code %d", ik_response.error_code.val);
    return false;
  }
  return true;
}

bool MechanismInterface::checkStateValidity(std::string arm_name, const std::vector<double> &joint_values,
                                          const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                            const std::vector<arm_navigation_msgs::LinkPadding> &link_padding)
{
  //prepare the planning scene
  getPlanningScene(collision_operations, link_padding);
  //call check state validity
  arm_navigation_msgs::GetStateValidity::Request req;
  arm_navigation_msgs::GetStateValidity::Response res;
  req.group_name = handDescription().armGroup(arm_name);
  if (!joint_values.empty())
  {
    req.robot_state.joint_state.name = getJointNames(arm_name);
    req.robot_state.joint_state.position = joint_values;
    if ( req.robot_state.joint_state.name.size() != joint_values.size() )
    {
      throw MechanismException("Wrong number of joint values for checkStateValidity");
    }
    req.robot_state.joint_state.header.stamp = ros::Time::now();
  }
  req.check_collisions = true;
  if(!check_state_validity_client_.client().call(req,res))
  {
    throw MechanismException("Call to check state validity client failed");
  }

  if (res.error_code.val == res.error_code.SUCCESS) return true;
  return false;
}

//! Finds the state of a given joint in the robot state
double getJointPosition( std::string name, const arm_navigation_msgs::RobotState& robot_state)
{
  for (size_t i=0; i<robot_state.joint_state.name.size(); i++)
  {
    if (robot_state.joint_state.name[i] == name)
    {
      ROS_ASSERT(robot_state.joint_state.position.size() > i);
      return robot_state.joint_state.position[i];
    }
  }
  ROS_ERROR_STREAM("Joint " << name << " not found in robot state");
  return 0.0;
}

/*! If starting_from_end is set to true, the Interpolated IK server will be set to start 
  from end as well, and the resulting trajectory is copied into the result starting from 
  the end until a failed step is encountered.

  - seed_joint_position is a seed to be use for IK for the joints we are planning on. Pass
  an empty vector if you don't have a seed you want to use

  - joint_state is a list of values to be used for joints that are not part of our plan.
  For example, use this to specify if you want the plan done with the gripper open
  or closed. If you don't specify a joint in here, the current value of that joint will
  be used by the Interpolated IK server.
 */
int MechanismInterface::getInterpolatedIK(std::string arm_name,
					  geometry_msgs::PoseStamped start_pose,
					  geometry_msgs::Vector3Stamped direction,
					  float desired_trajectory_length,
					  const std::vector<double> &seed_joint_position,
					  const sensor_msgs::JointState &joint_state,
					  const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
					  const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
					  bool reverse_trajectory,
					  trajectory_msgs::JointTrajectory &trajectory,
					  float &actual_trajectory_length)
{
  //first compute the desired end pose
  //make sure the input is normalized
  geometry_msgs::Vector3Stamped direction_norm = direction;
  direction_norm.vector = normalize(direction.vector);
  //multiply by the length
  desired_trajectory_length = fabs(desired_trajectory_length);
  direction_norm.vector.x *= desired_trajectory_length;
  direction_norm.vector.y *= desired_trajectory_length;
  direction_norm.vector.z *= desired_trajectory_length;
  geometry_msgs::PoseStamped end_pose = translateGripperPose(direction_norm, start_pose, arm_name);
 
  //hard-coded for now
  float max_step_size = 0.01;
  unsigned int collision_check_resolution = 2;
 
  //compute the number of steps  
  unsigned int num_steps = (unsigned int)ceil(desired_trajectory_length / fabs(max_step_size));
  float actual_step_size = desired_trajectory_length / num_steps;

  ROS_DEBUG_NAMED("manipulation","Trajectory details: length %f, requested num steps: %d, actual step size: %f",
	   desired_trajectory_length, num_steps, actual_step_size);

  if (reverse_trajectory)
  {
    std::swap(start_pose, end_pose);
  }

  //recall that here we setting the number of points in trajectory, which is steps+1
  setInterpolatedIKParams(arm_name, num_steps+1, collision_check_resolution, reverse_trajectory);

  arm_navigation_msgs::RobotState start_state;
  start_state.multi_dof_joint_state.child_frame_ids.push_back(handDescription().gripperFrame(arm_name));
  start_state.multi_dof_joint_state.poses.push_back(start_pose.pose);
  start_state.multi_dof_joint_state.frame_ids.push_back(start_pose.header.frame_id);
  start_state.multi_dof_joint_state.stamp = ros::Time::now();

  //pass the seeds for the IK
  if (!seed_joint_position.empty())
  { 
    //the caller has provided seeds for planned joints
    //we are silently assuming that the values passed in match our joint names for IK
    start_state.joint_state.name = getJointNames(arm_name);
    if (seed_joint_position.size() != start_state.joint_state.name.size())
    {
      ROS_ERROR("Interpolated IK request: seed_joint_position does not match joint names");
      throw MechanismException("Interpolated IK request: seed_joint_position does not match joint names");
    }
    start_state.joint_state.position = seed_joint_position;
  }
  else
  {
    //no seeds have been passed in, use current robot state
    //grab the latest JointState message and read in joint positions
    arm_navigation_msgs::RobotState robot_state;
    getRobotState(robot_state);
    start_state.joint_state.name = getJointNames(arm_name);
    for (size_t i=0; i<start_state.joint_state.name.size(); i++)
    {
      start_state.joint_state.position.push_back( getJointPosition(start_state.joint_state.name[i], robot_state) );
    }
  }

  //pass the desired values of non-planned joints, if any
  for (size_t i=0; i<joint_state.name.size(); i++)
  {
    start_state.joint_state.name.push_back(joint_state.name[i]);
    start_state.joint_state.position.push_back(joint_state.position[i]);
  }
  
  arm_navigation_msgs::PositionConstraint position_constraint;
  arm_navigation_msgs::OrientationConstraint orientation_constraint;
  arm_navigation_msgs::poseStampedToPositionOrientationConstraints(end_pose, handDescription().gripperFrame(arm_name),
								    position_constraint, 
								    orientation_constraint);
  arm_navigation_msgs::Constraints goal_constraints;
  goal_constraints.position_constraints.push_back(position_constraint);
  goal_constraints.orientation_constraints.push_back(orientation_constraint);

  arm_navigation_msgs::GetMotionPlan motion_plan;
  motion_plan.request.motion_plan_request.start_state = start_state;
  motion_plan.request.motion_plan_request.goal_constraints = goal_constraints;

  //prepare the planning scene
  getPlanningScene(collision_operations, link_padding);

  //PROF_COUNT(INTERPOLATED_IK);
  //PROF_START_TIMER(INTERPOLATED_IK);
  if ( !interpolated_ik_service_client_.client(arm_name).call(motion_plan) ) 
  {
    ROS_ERROR("  Call to Interpolated IK service failed");
    throw MechanismException("Call to Interpolated IK service failed");
  }
  //PROF_STOP_TIMER(INTERPOLATED_IK);

  trajectory.points.clear();
  trajectory.joint_names = motion_plan.response.trajectory.joint_trajectory.joint_names;

  if (motion_plan.response.trajectory_error_codes.empty()) 
  {
    ROS_ERROR("  Interpolated IK: empty trajectory received");
    throw MechanismException("Interpolated IK: empty trajectory received");
  }
  
  int error_code = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
  if (!reverse_trajectory)
  {
    for (size_t i=0; i<motion_plan.response.trajectory_error_codes.size(); i++) 
    {
      if ( motion_plan.response.trajectory_error_codes[i].val == motion_plan.response.trajectory_error_codes[i].SUCCESS )
      {
	trajectory.points.push_back( motion_plan.response.trajectory.joint_trajectory.points[i] );
      } 
      else 
      {
	ROS_DEBUG_NAMED("manipulation","  Interpolated IK failed on step %d (forward towards %d) with error code %d", 
		 (int) i, 
		 (int) motion_plan.response.trajectory_error_codes.size() - 1, 
		 motion_plan.response.trajectory_error_codes[i].val);
	error_code = motion_plan.response.trajectory_error_codes[i].val;
	break;
      }
    }
  }
  else
  {
    size_t first_success = 0;
    while ( first_success < motion_plan.response.trajectory_error_codes.size() &&
	    motion_plan.response.trajectory_error_codes[first_success].val != 
	    motion_plan.response.trajectory_error_codes[first_success].SUCCESS ) first_success ++;
    if (first_success != 0)
    {
      ROS_DEBUG_NAMED("manipulation","  Interpolation failed on step %d (backwards from %d) with error code %d",
	       (int) first_success - 1,
	       (int) motion_plan.response.trajectory_error_codes.size() - 1,
	       motion_plan.response.trajectory_error_codes[first_success - 1].val);
      error_code = motion_plan.response.trajectory_error_codes[first_success - 1].val;
    }
    else
    {
      ROS_DEBUG_NAMED("manipulation","  Interpolation trajectory complete (backwards from %d points)",
	       (int) motion_plan.response.trajectory_error_codes.size());
      
    }
    for (size_t i=first_success; i < motion_plan.response.trajectory_error_codes.size(); i++) 
    {
      if ( motion_plan.response.trajectory_error_codes[i].val == motion_plan.response.trajectory_error_codes[i].SUCCESS )
      {
	trajectory.points.push_back( motion_plan.response.trajectory.joint_trajectory.points[i] );
      } 
      else 
      {
	ROS_ERROR("  Interpolated IK: unexpected behavior for error codes: step %d has error code %d",
		  (int) i, motion_plan.response.trajectory_error_codes[i].val);
	throw MechanismException("Interpolated IK: unexpected behavior for error codes");
      }
    }
  }

  if (!trajectory.points.empty()) actual_trajectory_length = actual_step_size * (trajectory.points.size()-1);
  else actual_trajectory_length = 0.0;
  return error_code;
}

void MechanismInterface::cancelMoveArmGoal(std::string arm_name)
{
  move_arm_action_client_.client(arm_name).cancelGoal();
}

actionlib::SimpleClientGoalState MechanismInterface::getMoveArmState(std::string arm_name)
{
  actionlib::SimpleClientGoalState state = move_arm_action_client_.client(arm_name).getState();
  return state;
}

bool MechanismInterface::clearMoveArmGoals(double timeout)
{
  //if move_arm is running, cancel its goal
  actionlib::SimpleClientGoalState state = getMoveArmState("right_arm");
  if(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
  {
    ROS_INFO("move_arm for the right arm is running!  Attempting to cancel goals");
    ros::Time start_time = ros::Time::now();
    cancelMoveArmGoal("right_arm");
    while(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
    {
      ros::Duration(0.1).sleep();
      state = getMoveArmState("right_arm");
      if(ros::Time::now() - start_time > ros::Duration(timeout))
      {
        ROS_WARN("Timed out while waiting for move arm goal to finish");
        return false;
      } 
    }
  }
  state = getMoveArmState("left_arm");
  if(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
  {
    ROS_INFO("move_arm for the left arm is running!  Attempting to cancel goals");
    ros::Time start_time = ros::Time::now();
    cancelMoveArmGoal("left_arm");
    while(state == actionlib::SimpleClientGoalState::ACTIVE || state == actionlib::SimpleClientGoalState::PENDING)
    {
      ros::Duration(0.1).sleep();
      state = getMoveArmState("left_arm");
      if(ros::Time::now() - start_time > ros::Duration(timeout))
      {
        ROS_WARN("Timed out while waiting for move arm goal to finish");
        return false;
      } 
    }
  }
  ROS_INFO("Done clearing move arm goals");
  return true;
}

bool MechanismInterface::attemptMoveArmToGoal(std::string arm_name, const std::vector<double> &desired_joint_values,
                                              const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                              const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
                                              int max_tries, bool reset_map_if_stuck, double timeout) 
{
  //make sure joint controllers are running
  if(!checkController(jointControllerName(arm_name)))
     switchToJoint(arm_name);

  int num_tries = 0;  
  arm_navigation_msgs::MoveArmGoal move_arm_goal;
  move_arm_goal.planning_scene_diff.link_padding = link_padding;
  move_arm_goal.operations = collision_operations;

  move_arm_goal.motion_plan_request.group_name = handDescription().armGroup(arm_name);
  move_arm_goal.motion_plan_request.num_planning_attempts = 1;
  move_arm_goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  move_arm_goal.motion_plan_request.planner_id = MOVE_ARM_PLANNER_ID;
  move_arm_goal.planner_service_name = MOVE_ARM_PLANNER_SERVICE_NAME;
  

  move_arm_goal.operations = collision_operations;
  move_arm_goal.planning_scene_diff.link_padding = link_padding;

  std::vector<std::string> joint_names = getJointNames(arm_name);
  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints.resize(joint_names.size());
  for(unsigned int i = 0; i < desired_joint_values.size(); i++) 
  {
    move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names.at(i);
    move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[i].position = desired_joint_values[i];
    move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = .08;
    move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = .08;
  }
  
  bool success = false;
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  while(num_tries < max_tries)
  {
    move_arm_action_client_.client(arm_name).sendGoal(move_arm_goal);
    bool withinWait = move_arm_action_client_.client(arm_name).waitForResult(ros::Duration(timeout));
    if(!withinWait) 
    {
      move_arm_action_client_.client(arm_name).cancelGoal();
      ROS_DEBUG_NAMED("manipulation","   Move arm goal could not be achieved by move_arm in the allowed duration");
      success = false;
      num_tries++;
      arm_navigation_msgs::MoveArmResult move_arm_result = *move_arm_action_client_.client(arm_name).getResult();
      error_code.val = error_code.TIMED_OUT;
      modifyMoveArmGoal(move_arm_goal,error_code,move_arm_result.contacts);
      continue;
    }
    actionlib::SimpleClientGoalState state = move_arm_action_client_.client(arm_name).getState();
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      ROS_DEBUG_NAMED("manipulation","  Move arm: position was successfully achieved");
      success = true;
      break;
    } 
    else 
    {
      arm_navigation_msgs::MoveArmResult move_arm_result = *move_arm_action_client_.client(arm_name).getResult();
      ROS_DEBUG_NAMED("manipulation","   Move arm: non-success state was reached. Reason: %s",
               (arm_navigation_msgs::armNavigationErrorCodeToString(move_arm_result.error_code)).c_str());
      ROS_DEBUG_NAMED("manipulation","   num_tries: %d, max_tries: %d",num_tries,max_tries);
      success = false;
      num_tries++;
      error_code = move_arm_result.error_code;

      //reset the collision map and wait to repopulate it before trying again
      if(reset_map_if_stuck && error_code.val == error_code.START_STATE_IN_COLLISION && num_tries < max_tries)
      {
        std_srvs::Empty srv;
        if ( !reset_collision_map_service_.client().call(srv) )
        {
          ROS_ERROR("Mechanism interface: reset collision map service call failed");
        }
        //tried to reset and repopulate twice already; just reset and try again right away
        if(num_tries <= 2){
          ros::Duration(5.0).sleep();
          ROS_INFO("move arm is stuck!  Resetting the collision map and repopulating!");
        }
        else{
          ROS_ERROR("move arm is still stuck!  Resetting the collision map and NOT repopulating");
        }
      }
      //modifyMoveArmGoal(move_arm_goal,error_code,move_arm_result.contacts);
      continue;
    }
  }
  //! If move arm is still stuck, then nothing will get it out of here
  if (!success && error_code.val == error_code.START_STATE_IN_COLLISION)
  {
    ROS_ERROR("move arm is still stuck!  Throwing MoveArmStuckException!");
    throw MoveArmStuckException();
  }
  return success;
}


void MechanismInterface::modifyMoveArmGoal(arm_navigation_msgs::MoveArmGoal &move_arm_goal, 
                                           arm_navigation_msgs::ArmNavigationErrorCodes &error_code,
                                           std::vector<arm_navigation_msgs::ContactInformation> &contact_info_)
{
  double allowed_penetration_depth = 0.03;
  if(error_code.val == error_code.START_STATE_IN_COLLISION)
  {
    std::vector<arm_navigation_msgs::AllowedContactSpecification> allowed_contacts;
    for(unsigned int i=0; i < contact_info_.size(); i++)
    {
      if(contact_info_[i].depth < allowed_penetration_depth)
      {
        arm_navigation_msgs::AllowedContactSpecification allowed_contact_tmp;
        allowed_contact_tmp.shape.type = allowed_contact_tmp.shape.BOX;
        allowed_contact_tmp.shape.dimensions.resize(3);
        allowed_contact_tmp.shape.dimensions[0] = 0.03;
        allowed_contact_tmp.shape.dimensions[1] = 0.03;
        allowed_contact_tmp.shape.dimensions[2] = 0.03;
        allowed_contact_tmp.pose_stamped.pose.position = contact_info_[i].position;
        allowed_contact_tmp.pose_stamped.pose.orientation.x = 0.0;
        allowed_contact_tmp.pose_stamped.pose.orientation.y = 0.0;
        allowed_contact_tmp.pose_stamped.pose.orientation.z = 0.0;
        allowed_contact_tmp.pose_stamped.pose.orientation.w = 1.0;
        allowed_contact_tmp.pose_stamped.header.stamp = ros::Time::now();
        allowed_contact_tmp.pose_stamped.header.frame_id = contact_info_[i].header.frame_id;
        allowed_contact_tmp.link_names.push_back(contact_info_[i].contact_body_1);
        allowed_contact_tmp.penetration_depth = allowed_penetration_depth;
        allowed_contacts.push_back(allowed_contact_tmp);
        ROS_DEBUG_NAMED("manipulation","Added allowed contact region: %d",i);
        ROS_DEBUG_NAMED("manipulation","Position                    : (%f,%f,%f)",contact_info_[i].position.x,
		 contact_info_[i].position.y,contact_info_[i].position.z);
        ROS_DEBUG_NAMED("manipulation","Frame id                    : %s",contact_info_[i].header.frame_id.c_str());
        ROS_DEBUG_NAMED("manipulation","Depth                       : %f",allowed_penetration_depth);
        ROS_DEBUG_NAMED("manipulation","Link                        : %s",contact_info_[i].contact_body_1.c_str());
        ROS_DEBUG_NAMED("manipulation"," ");
      }
    }
    ROS_DEBUG_NAMED("manipulation","Added %d allowed contact regions",(int)allowed_contacts.size());
    move_arm_goal.planning_scene_diff.allowed_contacts = allowed_contacts;
  }
}


bool MechanismInterface::moveArmConstrained(std::string arm_name, const geometry_msgs::PoseStamped &commanded_pose,
                                            const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                            const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
                                            const arm_navigation_msgs::Constraints &path_constraints,
                                            const double &redundancy,
                                            const bool &compute_viable_command_pose)
{
  //make sure joint controllers are running
  if(!checkController(jointControllerName(arm_name)))
     switchToJoint(arm_name);

  int num_tries = 0;
  int max_tries = 1;

  arm_navigation_msgs::MoveArmGoal move_arm_goal;
  // no need to set the planning scene as move arm does not use that; instead
  // we'll set collision ops and link padding in move arm call
  move_arm_goal.planning_scene_diff.link_padding = link_padding;
  move_arm_goal.operations = collision_operations;

  move_arm_goal.motion_plan_request.group_name = handDescription().armGroup(arm_name)+"_cartesian";
  move_arm_goal.motion_plan_request.num_planning_attempts = 1;
  move_arm_goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  move_arm_goal.motion_plan_request.planner_id = MOVE_ARM_PLANNER_ID;
  move_arm_goal.planner_service_name = MOVE_ARM_CONSTRAINED_PLANNER_SERVICE_NAME;

  move_arm_goal.motion_plan_request.goal_constraints.position_constraints.resize(1);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = 
    commanded_pose.header.frame_id;
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = 
    handDescription().gripperFrame(arm_name);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].position = commanded_pose.pose.position;
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = 
    arm_navigation_msgs::Shape::BOX;
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].
    constraint_region_shape.dimensions.push_back(OBJECT_POSITION_TOLERANCE_X);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].
    constraint_region_shape.dimensions.push_back(OBJECT_POSITION_TOLERANCE_Y);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].
    constraint_region_shape.dimensions.push_back(OBJECT_POSITION_TOLERANCE_Z);
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
  move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;
  
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.resize(1);
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints[0] = 
    path_constraints.orientation_constraints[0];
  move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();

  tf::Quaternion orientation;
  tf::quaternionMsgToTF(commanded_pose.pose.orientation,orientation);
  geometry_msgs::PoseStamped desired_pose = commanded_pose;  
  desired_pose.pose.position = move_arm_goal.motion_plan_request.goal_constraints.position_constraints[0].position;
  desired_pose.header.stamp = ros::Time::now();

  // TODO - this should really happen through the constraint
  // Currently its hard-coded for keeping objects level
  if(compute_viable_command_pose && path_constraints.orientation_constraints.size())
  {
    bool get_ik = false;
    for(unsigned int i=0; i < 360; i++)
    {
      double rotation_yaw = i*M_PI/360.0;
      ROS_DEBUG_NAMED("manipulation","Trying to find suitable goal orientation with yaw rotation: %f",rotation_yaw);
      tf::Quaternion orientation_yaw;
      orientation_yaw.setRPY(0.0,0.0,rotation_yaw);
      orientation_yaw *= orientation;
      geometry_msgs::Quaternion quaternion;
      tf::quaternionTFToMsg(orientation_yaw,quaternion);
      desired_pose.pose.orientation = quaternion;
      desired_pose.header.stamp = ros::Time::now();
      kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response;
      if(getIKForPose(arm_name,desired_pose,ik_response, collision_operations, link_padding))
      {
        get_ik = true;
        break;
      }
    }
    if(!get_ik) 
      return false;
  }

  move_arm_goal.motion_plan_request.path_constraints.orientation_constraints.resize(1);
  move_arm_goal.motion_plan_request.path_constraints.orientation_constraints[0] = 
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints[0];
  move_arm_goal.disable_ik = true;

  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints.resize(1);
  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[0].joint_name = (getJointNames(arm_name))[2];
  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[0].position = redundancy;
  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[0].tolerance_below = M_PI;
  move_arm_goal.motion_plan_request.goal_constraints.joint_constraints[0].tolerance_above = M_PI;
      
  bool success = false;
  while(num_tries < max_tries)
  {
    move_arm_action_client_.client(arm_name).sendGoal(move_arm_goal);
    bool withinWait = move_arm_action_client_.client(arm_name).waitForResult(ros::Duration(60.0));
    if(!withinWait) 
    {
      move_arm_action_client_.client(arm_name).cancelGoal();
      ROS_DEBUG_NAMED("manipulation","  Move arm goal could not be achieved by move_arm in the allowed duration");
      success = false;
      num_tries++;
      arm_navigation_msgs::MoveArmResult move_arm_result = *move_arm_action_client_.client(arm_name).getResult();
      arm_navigation_msgs::ArmNavigationErrorCodes error_code;
      error_code.val = error_code.TIMED_OUT;
      modifyMoveArmGoal(move_arm_goal,error_code,move_arm_result.contacts);
      continue;
    }
    actionlib::SimpleClientGoalState state = move_arm_action_client_.client(arm_name).getState();
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      ROS_DEBUG_NAMED("manipulation","  Move arm: position was successfully achieved");
      success = true;
      break;
    } 
    else 
    {
      arm_navigation_msgs::MoveArmResult move_arm_result = *move_arm_action_client_.client(arm_name).getResult();
      ROS_DEBUG_NAMED("manipulation","Move arm: non-success state was reached. Reason: %s",
               (arm_navigation_msgs::armNavigationErrorCodeToString(move_arm_result.error_code)).c_str());
      ROS_DEBUG_NAMED("manipulation","num_tries: %d, max_tries: %d",num_tries,max_tries);
      success = false;
      num_tries++;
      modifyMoveArmGoal(move_arm_goal,move_arm_result.error_code,move_arm_result.contacts);
      continue;
    }
  }
  return success;
}

/*! The translation must be expressed either in the gripper frame or in the robot frame. The reason is the 
  following: this function must work even without tf, so that it works for hypothetical situations,
  or situations in which the complete tf tree is unknown. Its only decision is thus whether to
  pre-multiply or post-multiply the pose with the given translation. 

  If the translation is in gripper frame, the pose gets post-multiplied by it.
  If the translation is in robot frame, the pose gets pre-multiplied by it.

  The input pose is first transformed to the robot frame, then translated, then transformed back into
  the original frame.
 */
geometry_msgs::PoseStamped MechanismInterface::translateGripperPose(geometry_msgs::Vector3Stamped translation, 
								    geometry_msgs::PoseStamped start_pose,
								    std::string arm_name)
{
  bool pre_multiply;
  if (translation.header.frame_id == handDescription().gripperFrame(arm_name))
  {
    pre_multiply=false;
  } 
  else if (translation.header.frame_id == handDescription().robotFrame(arm_name))
  {
    pre_multiply=true;
  }
  else
  {
    throw MechanismException(std::string("Gripper translations (such as for lift or approach) can only be specified in "
				     "either the gripper frame or the robot frame. This one was specified in frame ") + 
			     translation.header.frame_id);
  }


  //go to robot frame first
  geometry_msgs::PoseStamped start_pose_robot_frame = transformPose(handDescription().robotFrame(arm_name), start_pose);
  tf::StampedTransform start_transform;
  tf::poseMsgToTF(start_pose_robot_frame.pose, start_transform);

  tf::Vector3 vec;
  tf::vector3MsgToTF(translation.vector, vec);
  tf::StampedTransform translate_trans;
  translate_trans.setIdentity();
  translate_trans.setOrigin(vec);

  //compute the translated pose
  tf::Transform end_transform;
  if (pre_multiply) end_transform = translate_trans * start_transform;
  else end_transform = start_transform * translate_trans;

  //prepare the results
  geometry_msgs::PoseStamped translated_pose;
  tf::poseTFToMsg(end_transform, translated_pose.pose);
  translated_pose.header.frame_id = handDescription().robotFrame(arm_name);
  translated_pose.header.stamp = ros::Time(0);

  //return the result in the requested frame
  translated_pose = transformPose(start_pose.header.frame_id, translated_pose);

  return translated_pose;
}

/*! Current gripper pose is returned in the requested frame_id.*/
geometry_msgs::PoseStamped MechanismInterface::getGripperPose(std::string arm_name, std::string frame_id)
{
  tf::StampedTransform gripper_transform;
  try
  {
    listener_.lookupTransform(frame_id, handDescription().gripperFrame(arm_name), ros::Time(0), gripper_transform);
  }
  catch (tf::TransformException ex)
  {
    //sometimes fails the first time but succeeds the second.  Don't ask me why.
    ROS_ERROR("failed to get tf transform for wrist roll link, trying a second time");
    try
    {
      listener_.lookupTransform(frame_id, handDescription().gripperFrame(arm_name), ros::Time(0), gripper_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Mechanism interface: failed to get tf transform for wrist roll link; tf exception %s", ex.what());
      throw MechanismException(std::string("failed to get tf transform for wrist roll link; tf exception: ") + 
			     std::string(ex.what()) );
    }
  }
  geometry_msgs::PoseStamped gripper_pose;
  tf::poseTFToMsg(gripper_transform, gripper_pose.pose);
  gripper_pose.header.frame_id = frame_id;
  gripper_pose.header.stamp = ros::Time::now();
  return gripper_pose;
}

void MechanismInterface::transformPointCloud(std::string target_frame, 
					     const sensor_msgs::PointCloud &cloud_in,
					     sensor_msgs::PointCloud &cloud_out)
{
  try
  {
    listener_.transformPointCloud(target_frame, cloud_in, cloud_out);    
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Mechanism interface: failed to cloud into %s frame; exception: %s", target_frame.c_str(),
	      ex.what());
    throw MechanismException(std::string("failed to transform cloud into frame ") + target_frame + 
			     std::string("; tf exception: ") + std::string(ex.what()) );
  }
}

void MechanismInterface::convertGraspableObjectComponentsToFrame(object_manipulation_msgs::GraspableObject &object,
                                                                 std::string frame_id)
{
  if (!object.cluster.points.empty())
  {
    transformPointCloud(frame_id, object.cluster, object.cluster);
  }
  for (size_t i=0; i<object.potential_models.size(); i++)
  {
    object.potential_models[i].pose = transformPose(frame_id, object.potential_models[i].pose);
  }
  object.reference_frame_id = frame_id;
}

geometry_msgs::PoseStamped MechanismInterface::transformPose(const std::string target_frame, 
							     const geometry_msgs::PoseStamped &stamped_in)
{
  geometry_msgs::PoseStamped stamped_out;
  try
  {
    listener_.transformPose(target_frame, stamped_in, stamped_out);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Mechanism interface: failed to transform pose into %s frame; exception: %s", target_frame.c_str(),
	      ex.what());
    throw MechanismException(std::string("failed to transform pose into frame ") + target_frame + 
			     std::string("; tf exception: ") + std::string(ex.what()) );
  }
  return stamped_out;
}

/*! Moves the gripper from its current pose to the one obtained by the specified translation.
*/
bool MechanismInterface::translateGripper(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
					  arm_navigation_msgs::OrderedCollisionOperations ord, 
					  const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
					  float requested_distance, float min_distance,
					  float &actual_distance)
{
  //get the current gripper pose in the robot frame
  geometry_msgs::PoseStamped start_pose_stamped = getGripperPose(arm_name, handDescription().robotFrame(arm_name));

  //compute the interpolated trajectory
  trajectory_msgs::JointTrajectory traj;
  std::vector<double> empty;
  sensor_msgs::JointState empty_state;
  getInterpolatedIK(arm_name, 
		    start_pose_stamped, direction, requested_distance, 
		    empty, empty_state, ord, link_padding, false, traj, actual_distance);

  if (min_distance > 0 && actual_distance < min_distance) 
  {
    ROS_DEBUG_NAMED("manipulation","Mechanism interface: interpolated IK trajectory covers %f distance, but at least %f requested", 
	     actual_distance, min_distance);
    actual_distance = 0;
    return false;
  }

  if (traj.points.empty())
  {
    ROS_DEBUG_NAMED("manipulation","Mechanism interface: translate gripper trajectory is empty");
    actual_distance = false;
    return false;
  }

  //execute the normalized interpolated trajectory
  attemptTrajectory(arm_name, traj, true);
  return true;
}

/*! Object pose is obtained by multiplying the grasp_pose with the current pose
  of the wrist roll link of the given arm. Result is given in base_link frame.
*/
geometry_msgs::PoseStamped MechanismInterface::getObjectPoseForGrasp(std::string arm_name, 
								     const geometry_msgs::Pose &grasp_pose)

{
  //get the current pose of the gripper in base link coordinate frame
  tf::StampedTransform wrist_transform;
  try
  {
    listener_.lookupTransform("base_link",handDescription().gripperFrame(arm_name), 
			      ros::Time(0), wrist_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Mechanism interface: failed to get tf transform for wrist roll link");
    throw MechanismException(std::string("failed to get tf transform for wrist roll link; tf exception: ") + 
			     std::string(ex.what()) );
  }

  //multiply by inverse of grasp pose
  tf::Transform grasp_transform;
  tf::poseMsgToTF(grasp_pose, grasp_transform);

  tf::Transform object_transform;
  object_transform = wrist_transform * grasp_transform.inverse();

  //prepare the result
  geometry_msgs::PoseStamped object_pose;
  tf::poseTFToMsg(object_transform, object_pose.pose);
  object_pose.header.frame_id = "base_link";
  object_pose.header.stamp = ros::Time::now();
  return object_pose;
}

void MechanismInterface::attachObjectToGripper(std::string arm_name, std::string collision_object_name)
{
  arm_navigation_msgs::AttachedCollisionObject obj;
  obj.object.header.stamp = ros::Time::now();
  obj.object.header.frame_id = handDescription().robotFrame(arm_name);
  obj.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT;
  obj.object.id = collision_object_name;
  obj.link_name = handDescription().attachLinkName(arm_name);
  obj.touch_links = handDescription().gripperTouchLinkNames(arm_name);
  attached_object_pub_.publish(obj);
}
 
void MechanismInterface::detachAndAddBackObjectsAttachedToGripper(std::string arm_name, 
								  std::string collision_object_name)
{
  arm_navigation_msgs::AttachedCollisionObject att;
  att.object.header.stamp = ros::Time::now();
  att.object.header.frame_id = handDescription().robotFrame(arm_name);
  att.link_name = handDescription().attachLinkName(arm_name);
  att.object.id = collision_object_name;
  att.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT;
  attached_object_pub_.publish(att);
}

void MechanismInterface::detachAllObjectsFromGripper(std::string arm_name)
{
  arm_navigation_msgs::AttachedCollisionObject att;
  att.object.header.stamp = ros::Time::now();
  att.object.header.frame_id = handDescription().robotFrame(arm_name);
  att.link_name = handDescription().attachLinkName(arm_name);
  att.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  attached_object_pub_.publish(att);
}

void MechanismInterface::handPostureGraspAction(std::string arm_name, 
                    const object_manipulation_msgs::Grasp &grasp, int goal, float max_contact_force)
{
  object_manipulation_msgs::GraspHandPostureExecutionGoal posture_goal;
  posture_goal.grasp = grasp;
  posture_goal.goal = goal;
  posture_goal.max_contact_force = max_contact_force;
  hand_posture_client_.client(arm_name).sendGoal(posture_goal);
  bool withinWait = hand_posture_client_.client(arm_name).waitForResult(ros::Duration(10.0));
  if(!withinWait) 
  {
    hand_posture_client_.client(arm_name).cancelGoal();
    ROS_ERROR("Hand posture controller timed out on goal (%d)", goal);
    throw MechanismException("Hand posture controller timed out");
  }
  actionlib::SimpleClientGoalState state = hand_posture_client_.client(arm_name).getState();
  if(state != actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    ROS_ERROR("Hand posture controller failed on goal (%d)", goal);
    throw MechanismException("Hand posture controller failed");
  }
  ROS_DEBUG_NAMED("manipulation","Hand posture controller successfully achieved goal %d", goal);
}

bool MechanismInterface::graspPostureQuery(std::string arm_name, const object_manipulation_msgs::Grasp grasp)
{
  object_manipulation_msgs::GraspStatus query;
  query.request.grasp = grasp;
  if (!grasp_status_client_.client(arm_name).call(query))
  {
    ROS_ERROR("Grasp posture query call failed");
    throw MechanismException("Grasp posture query call failed");
  }
  return query.response.is_hand_occupied;
}

bool MechanismInterface::pointHeadAction(const geometry_msgs::PointStamped &target, std::string pointing_frame, bool wait_for_result)
{
  pr2_controllers_msgs::PointHeadGoal goal;
  goal.target = target;
  goal.pointing_axis.x = 0;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 1;
  goal.pointing_frame = pointing_frame;
  goal.min_duration = ros::Duration(0.05);
  goal.max_velocity = 1.0;

  point_head_action_client_.client().sendGoal(goal);

  if(wait_for_result)
  {
    point_head_action_client_.client().waitForResult( ros::Duration(3.0) );

    if (point_head_action_client_.client().getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_DEBUG_NAMED("manipulation","Successfully moved head.");
      return true;
    }
    else
    {
      ROS_ERROR("Head movement failed or timed out.");
      return false;
    }
  }
  return true;
}

bool MechanismInterface::callSwitchControllers(std::vector<std::string> start_controllers, 
                                               std::vector<std::string> stop_controllers)
{
  pr2_mechanism_msgs::SwitchController srv;
  srv.request.start_controllers = start_controllers;
  srv.request.stop_controllers = stop_controllers;
  srv.request.strictness = srv.request.STRICT;
  if ( !switch_controller_service_.client().call(srv) )
  {
    ROS_ERROR("Mechanism interface: switch controller service call failed");
    throw MechanismException("switch controller service call failed");
  } 
  if(!srv.response.ok) return false;
  return true;
}  

bool MechanismInterface::switchControllers(std::string start_controller, std::string stop_controller)
{
  ROS_DEBUG_NAMED("manipulation","Switching controller %s for %s", start_controller.c_str(), stop_controller.c_str());
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  start_controllers.push_back(start_controller);
  stop_controllers.push_back(stop_controller);
  bool success = callSwitchControllers(start_controllers, stop_controllers);
  if(success)
  {
    bool start_running = checkController(start_controller);
    bool stop_running = checkController(stop_controller);
    if(start_running && !stop_running) return true;
    ROS_ERROR("switching %s to %s failed even though it returned success", 
              stop_controller.c_str(), start_controller.c_str());
    return false;
  }
  else
  {
    ROS_ERROR("switching %s to %s failed", stop_controller.c_str(), start_controller.c_str());
    return false;
  }
}

bool MechanismInterface::checkController(std::string controller)
{
  pr2_mechanism_msgs::ListControllers srv;
  if( !list_controllers_service_.client().call(srv))
  {
    ROS_ERROR("Mechanism interface: list controllers service call failed");
    throw MechanismException("list controllers service call failed");
  }
  for(size_t controller_ind=0; controller_ind < srv.response.controllers.size(); controller_ind++)
  {
    if(controller.compare(srv.response.controllers[controller_ind]) == 0)
    {
      if(srv.response.state[controller_ind].compare("running") == 0) return true;
      return false;
    }
  }
  ROS_WARN("controller %s not found when checking status!", controller.c_str());
  return false;
}

bool MechanismInterface::startController(std::string controller)
{
  ROS_DEBUG_NAMED("manipulation","Starting controller %s", controller.c_str());
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  start_controllers.push_back(controller);
  bool success = callSwitchControllers(start_controllers, stop_controllers);
  if(success)
  {
    bool running = checkController(controller);
    if(running) return true;
    ROS_ERROR("starting controller %s failed even though it returned success", controller.c_str());
    return false;
  }
  else
  {
    ROS_ERROR("starting controller %s failed", controller.c_str());
    return false;
  }
}

bool MechanismInterface::stopController(std::string controller)
{
  ROS_DEBUG_NAMED("manipulation","Stopping controller %s", controller.c_str());
  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  stop_controllers.push_back(controller);
  bool success = callSwitchControllers(start_controllers, stop_controllers);
  if(success)
  {
    bool running = checkController(controller);
    if(running)
    {
      ROS_ERROR("stopping controller %s failed even though it returned success", controller.c_str());
      return false;
    }
    return true;
  }
  else
  {
    ROS_ERROR("stopping controller %s failed", controller.c_str());
    return false;
  }
}

bool MechanismInterface::getArmAngles(std::string arm_name, std::vector<double> &arm_angles)
{
  std::vector<std::string> arm_joints = handDescription().armJointNames(arm_name);
  if(arm_joints.size() == 0)
  {
    ROS_ERROR("mechanism interface: armJointNames was empty!");
    return false;
  }
  arm_angles.resize(arm_joints.size());

  //grab the latest JointState message
  arm_navigation_msgs::RobotState robot_state;
  getRobotState(robot_state);
  //find each joint and its current position in the JointState message
  for (size_t joint_num = 0; joint_num < arm_joints.size(); joint_num++)
  {
    std::string joint_name = arm_joints[joint_num];
    size_t i;
    for (i=0; i<robot_state.joint_state.name.size(); i++)
    {
      if (robot_state.joint_state.name[i] == joint_name) break;
    }
    if (i==robot_state.joint_state.name.size())
    {
      ROS_ERROR("mechanism interface: arm joint %s not found in joint state", joint_name.c_str());
      return false;
    }
    if (robot_state.joint_state.position.size() <= i)
    {
      ROS_ERROR("mechanism interface: malformed joint state message received");
      return false;
    }
    arm_angles[joint_num] = robot_state.joint_state.position[i];
  }
  ROS_DEBUG_NAMED("manipulation","%s arm_angles: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
           arm_name.c_str(), arm_angles[0], arm_angles[1], arm_angles[2], 
           arm_angles[3], arm_angles[4], arm_angles[5], arm_angles[6]);
  return true;
}

bool MechanismInterface::switchToCartesian(std::string arm_name)
{
  //set the desired posture to the current joint angles first
  setCartesianPostureGoalToCurrentAngles(arm_name);
  
  //switch controllers
  bool result = switchControllers(cartesianControllerName(arm_name), jointControllerName(arm_name));
  if(!result) return false;
  return true;
}

bool MechanismInterface::switchToJoint(std::string arm_name)
{
  bool result = switchControllers(jointControllerName(arm_name), cartesianControllerName(arm_name));
  if(!result) return false;
  return true;
}

std::string MechanismInterface::jointControllerName(std::string arm_name)
{
  std::map<std::string, std::string>::iterator it = joint_controller_names_.find(arm_name);
  if ( it != joint_controller_names_.end() ) 
  {
    return it->second;
  }
  std::string controller_name;
  priv_nh_.param<std::string>(arm_name+std::string("_joint_controller"), controller_name, "");
  if(controller_name == "")
  {
    ROS_WARN("joint controller name for arm %s not found!", arm_name.c_str());
  }
  else
  {
    joint_controller_names_.insert(std::pair<std::string, std::string>(arm_name, controller_name));
    ROS_INFO("added controller name %s to joint controller map", controller_name.c_str());
  }
  return controller_name;
}

std::string MechanismInterface::cartesianControllerName(std::string arm_name)
{
  std::map<std::string, std::string>::iterator it = cartesian_controller_names_.find(arm_name);
  if ( it != cartesian_controller_names_.end() ) 
  {
    return it->second;
  }
  std::string controller_name;
  priv_nh_.param<std::string>(arm_name+std::string("_cartesian_controller"), controller_name, "");
  if(controller_name == "")
  {
    ROS_WARN("Cartesian controller name for arm %s not found!", arm_name.c_str());
  }
  else
  {
    cartesian_controller_names_.insert(std::pair<std::string, std::string>(arm_name, controller_name));
    ROS_INFO("added controller name %s to Cartesian controller map", controller_name.c_str());
  }
  return controller_name;
}

geometry_msgs::PoseStamped MechanismInterface::clipDesiredPose(std::string arm_name,
                                                               const geometry_msgs::PoseStamped &desired_pose,
                                                               double clip_dist, double clip_angle,
                                                               double &resulting_clip_fraction)
{
  //no clipping desired
  if(clip_dist == 0 && clip_angle == 0) return desired_pose;

  //Get the current gripper pose
  geometry_msgs::PoseStamped current_pose = getGripperPose(arm_name, desired_pose.header.frame_id);

  return clipDesiredPose(current_pose, desired_pose, clip_dist, clip_angle, resulting_clip_fraction);
}

geometry_msgs::PoseStamped MechanismInterface::clipDesiredPose(const geometry_msgs::PoseStamped &current_pose,
                                                               const geometry_msgs::PoseStamped &desired_pose,
                                                               double clip_dist, double clip_angle,
                                                               double &resulting_clip_fraction)
{
  //no clipping desired
  if(clip_dist == 0 && clip_angle == 0) return desired_pose;

  //Get the position and angle dists between current and desired
  Eigen::Affine3d current_trans, desired_trans;
  double pos_dist, angle;
  Eigen::Vector3d axis, direction;
  tf::poseMsgToEigen(current_pose.pose, current_trans);
  tf::poseMsgToEigen(desired_pose.pose, desired_trans);
  positionAndAngleDist(current_trans, desired_trans, pos_dist, angle, axis, direction);

  //Clip the desired pose to be at most clip_dist and the desired angle to be at most clip_angle (proportional)
  //from the current
  double pos_mult, angle_mult;
  double pos_change, angle_change;
  angle_mult = fabs(angle / clip_angle);
  pos_mult = fabs(pos_dist / clip_dist);
  if(pos_mult <=1 && angle_mult <=1){
    return desired_pose;
  }
  double mult = pos_mult;
  if(angle_mult > pos_mult) mult = angle_mult;
  pos_change = pos_dist / mult;
  angle_change = angle / mult;
  resulting_clip_fraction = 1 / mult;

  Eigen::Affine3d clipped_trans;
  clipped_trans = current_trans;
  Eigen::Vector3d scaled_direction;
  scaled_direction = direction * pos_change;
  Eigen::Translation3d translation(scaled_direction);
  clipped_trans = clipped_trans * translation;
  Eigen::AngleAxis<double> angle_axis(angle_change, axis);
  clipped_trans = clipped_trans * angle_axis;
  geometry_msgs::PoseStamped clipped_pose;
  tf::poseEigenToMsg(clipped_trans, clipped_pose.pose);
  clipped_pose.header = desired_pose.header;
  return clipped_pose;
}


geometry_msgs::PoseStamped MechanismInterface::overshootDesiredPose(std::string arm_name, 
								    const geometry_msgs::PoseStamped &desired_pose, 
								    double overshoot_dist, double overshoot_angle,
								    double dist_tol, double angle_tol)
{
  //Get the current gripper pose
  geometry_msgs::PoseStamped current_pose = getGripperPose(arm_name, desired_pose.header.frame_id);

  //Get the position and angle dists between current and desired
  Eigen::Affine3d current_trans, desired_trans;
  double pos_dist, angle;
  Eigen::Vector3d axis, direction;
  tf::poseMsgToEigen(current_pose.pose, current_trans);
  tf::poseMsgToEigen(desired_pose.pose, desired_trans);
  positionAndAngleDist(current_trans, desired_trans, pos_dist, angle, axis, direction);

  //Add overshoot_dist in length to the pos difference vector, and overshoot_angle to the angle difference
  double pos_change = 0;
  double angle_change = 0;
  if(angle > angle_tol) angle_change = angle + overshoot_angle;
  double pos_sign = pos_dist / fabs(pos_dist);
  if(fabs(pos_dist) > dist_tol) pos_change = pos_dist + pos_sign*overshoot_dist;
  //std::cout << "pos_change: " << pos_change << "angle_change: " << angle_change;

  Eigen::Affine3d overshoot_trans;
  overshoot_trans = current_trans;
  Eigen::Vector3d scaled_direction;
  scaled_direction = direction * pos_change;
  Eigen::Translation3d translation(scaled_direction);
  overshoot_trans = overshoot_trans * translation;
  Eigen::AngleAxis<double> angle_axis(angle_change, axis);
  overshoot_trans = overshoot_trans * angle_axis;
  geometry_msgs::PoseStamped overshoot_pose;
  tf::poseEigenToMsg(overshoot_trans, overshoot_pose.pose);
  overshoot_pose.header = desired_pose.header;

  return overshoot_pose;
} 


geometry_msgs::PoseStamped MechanismInterface::clipDesiredPose(std::string arm_name, 
                                                               const geometry_msgs::PoseStamped &desired_pose, 
                                                               double clip_dist, double clip_angle,
                                                               double &resulting_clip_fraction,
                                                               const std::vector<double> &goal_posture_suggestion,
                                                               std::vector<double> &clipped_posture_goal)
{
  double clip_fraction;
  geometry_msgs::PoseStamped clipped_pose = clipDesiredPose(arm_name, desired_pose, clip_dist, clip_angle, clip_fraction);

  //If there's a goal posture suggestion, clip it as well
  if(goal_posture_suggestion.size() > 0 && (clip_dist != 0 || clip_angle != 0))
  {
    //Get the current arm angles
    std::vector<double> current_arm_angles;
    getArmAngles(arm_name, current_arm_angles);

    if(goal_posture_suggestion.size() != current_arm_angles.size())
    {
      ROS_ERROR("goal posture suggestion length not consistent with length of current arm angles!");
      return clipped_pose;
    }

    //Unnormalize the goal arm angles
    std::vector<std::vector<double> > goal_positions;
    goal_positions.push_back(goal_posture_suggestion);
    trajectory_msgs::JointTrajectory goal_trajectory = assembleJointTrajectory(arm_name, goal_positions, 5.0);
    goal_trajectory.header.stamp = ros::Time(0);
    trajectory_msgs::JointTrajectory unnormalized_goal;
    unnormalizeTrajectory(arm_name, goal_trajectory, unnormalized_goal);

    //Clip the posture goal by the same amount the pose got clipped
    clipped_posture_goal.resize(current_arm_angles.size());
    for(size_t i=0; i<current_arm_angles.size(); i++)
    {
      clipped_posture_goal[i] = 
      	(unnormalized_goal.points[0].positions[i] - current_arm_angles[i]) * clip_fraction + current_arm_angles[i];
    }
  }

  return clipped_pose;
}

void MechanismInterface::poseDists(geometry_msgs::Pose start, geometry_msgs::Pose end, double &pos_dist, double &angle)
{
  Eigen::Affine3d start_trans, end_trans;
  tf::poseMsgToEigen(start, start_trans);
  tf::poseMsgToEigen(end, end_trans);
  Eigen::Vector3d axis, direction;
  positionAndAngleDist(start_trans, end_trans, pos_dist, angle, axis, direction);
}

void MechanismInterface::positionAndAngleDist(Eigen::Affine3d start, Eigen::Affine3d end, 
                                              double &pos_dist, 
					      double &angle, Eigen::Vector3d &axis, Eigen::Vector3d &direction)
{
  //trans = end to start = global to start * end to global
  Eigen::Affine3d trans;
  trans = start.inverse() * end;
  Eigen::AngleAxis<double> angle_axis;
  angle_axis = trans.rotation();
  angle = angle_axis.angle();
  axis = angle_axis.axis();
  if(angle > M_PI)
  {
    angle = -(angle - 2*M_PI);
    axis = -axis;
  }
  direction = trans.translation();
  pos_dist = sqrt(direction.dot(direction));
  if(pos_dist) direction *= 1/pos_dist;
}

int MechanismInterface::translateGripperCartesian(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
						  ros::Duration timeout, double dist_tol, double angle_tol,
						  double clip_dist, double clip_angle, 
						  double overshoot_dist, double overshoot_angle, double timestep)
{
  geometry_msgs::PoseStamped current_pose = getGripperPose(arm_name, direction.header.frame_id);
  geometry_msgs::PoseStamped desired_pose = translateGripperPose(direction, current_pose, arm_name);  
  int result = moveArmToPoseCartesian(arm_name, desired_pose, timeout, dist_tol, angle_tol, 
                                      clip_dist, clip_angle, overshoot_dist, overshoot_angle, timestep);
  return result;
}

//returns 0 if an error occurred, 1 if it got there, -1 if it timed out
int MechanismInterface::moveArmToPoseCartesian(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
					       ros::Duration timeout, double dist_tol, double angle_tol,
					       double clip_dist, double clip_angle, 
					       double overshoot_dist, double overshoot_angle, 
					       double timestep, 
					       const std::vector<double> &goal_posture_suggestion)
{
  bool success = false;

  //Switch to Cartesian controllers
  for(int tries=0; tries<3; tries++)
  {
    success = switchToCartesian(arm_name);
    if(success) break;
    ros::Duration(1.0).sleep();
  }
  if(!success)
  {
    ROS_ERROR("Tries exceeded when trying to switch to Cartesian control!");
    return 0;
  }

  //Move towards the desired pose until we get there within our tolerance or until time runs out
  ros::Time begin = ros::Time::now();
  int reached_target = -1;
  geometry_msgs::PoseStamped current_pose = getGripperPose(arm_name, desired_pose.header.frame_id);
  geometry_msgs::PoseStamped last_pose = current_pose;

  //Compute a new desired_pose with the desired overshoot
  geometry_msgs::PoseStamped overshoot_pose = overshootDesiredPose(arm_name, desired_pose, overshoot_dist, overshoot_angle, dist_tol, angle_tol);

  while(ros::ok())
  {
    //stop if we're out of time
    if(ros::Time::now() - begin > timeout)
    {
      ROS_DEBUG_NAMED("manipulation","Timed out when moving to desired Cartesian pose");
      break;
    }
    
    //stop if we're within our tolerances
    current_pose = getGripperPose(arm_name, desired_pose.header.frame_id);

    double pos_dist, angle_dist;
    poseDists(current_pose.pose, overshoot_pose.pose, pos_dist, angle_dist);
    if(pos_dist <= dist_tol+overshoot_dist && angle_dist <= angle_tol+overshoot_angle)
    {
      //send the original desired pose out
      sendCartesianPoseCommand(arm_name, desired_pose);

      //ROS_INFO("moveArmToPoseCartesian reached target");
      reached_target = 1;
      break;
    }

    //also stop if we're not moving
    double pos_change, angle_change;
    poseDists(current_pose.pose, last_pose.pose, pos_change, angle_change);
    if(ros::Time::now() - begin > ros::Duration(2.0) && pos_change <= .005*timestep && angle_change <= .001*timestep)
    {
      ROS_INFO("moveArmToPoseCartesian is stuck!  Returning");
      break;
    }
     
    //clip the desired pose and posture suggestion and send it out
    double resulting_clip_fraction;
    std::vector<double> clipped_posture_suggestion;
    geometry_msgs::PoseStamped clipped_pose = clipDesiredPose(arm_name, overshoot_pose, clip_dist, clip_angle, 
                                    resulting_clip_fraction, goal_posture_suggestion, clipped_posture_suggestion);
    sendCartesianPoseCommand(arm_name, clipped_pose);

    if(clipped_posture_suggestion.size() > 0)
    {
      sendCartesianPostureCommand(arm_name, clipped_posture_suggestion);
    }

    last_pose = current_pose;

    //sleep until the next timestep
    ros::Duration(timestep).sleep();
  }

  //Switch back to joint control
  success = false;
  for(int tries=0; tries<3; tries++)
  {
    success = switchToJoint(arm_name);
    if(success) break;
    ros::Duration(1.0).sleep();
  }
  if(!success)
  {
    ROS_ERROR("Tries exceeding when trying to switch back to joint control!");
    return 0;
  }
  return reached_target;
}

void MechanismInterface::sendCartesianPoseCommand(std::string arm_name, geometry_msgs::PoseStamped desired_pose)
{
  desired_pose.header.stamp = ros::Time(0);
  cartesian_pub_.publisher(arm_name).publish(desired_pose);
}

void MechanismInterface::sendCartesianPostureCommand(std::string arm_name, std::vector<double> arm_angles)
{
  std_msgs::Float64MultiArray angles;
  for(size_t i=0; i<arm_angles.size(); i++)
  {
    angles.data.push_back(arm_angles[i]);
  }
  cartesian_posture_pub_.publisher(arm_name).publish(angles);
}

/*
void MechanismInterface::sendCartesianGainsCommand(std::string arm_name, std::vector<double> gains)
{
  std_msgs::Float64MultiArray new_gains;
  new_gains.data = gains;
  cartesian_gains_pub_.publisher(arm_name).publish(new_gains);
  }*/
 /*
void MechanismInterface::sendCartesianGainsCommand(std::string arm_name, std::vector<double> gains, std::string frame_id, std::vector<double> fixed_frame)
{
  object_manipulation_msgs::CartesianGains new_gains;
  new_gains.gains = gains;
  new_gains.fixed_frame = fixed_frame;
  new_gains.header.frame_id = frame_id;
  cartesian_gains_pub_.publisher(arm_name).publish(new_gains);
  }*/

void MechanismInterface::setCartesianPostureGoalToCurrentAngles(std::string arm_name)
{
  std::vector<double> arm_angles;
  getArmAngles(arm_name, arm_angles);
  sendCartesianPostureCommand(arm_name, arm_angles);
}

std::vector<arm_navigation_msgs::LinkPadding> 
MechanismInterface::fingertipPadding(std::string arm_name, double pad)
{
  std::vector<arm_navigation_msgs::LinkPadding> padding_vec;
  arm_navigation_msgs::LinkPadding padding;
  padding.padding = pad;  
  std::vector<std::string> links = handDescription().fingertipLinks(arm_name);
  for (size_t i=0; i<links.size(); i++)
  {
    padding.link_name = links[i];
    padding_vec.push_back(padding); 
  }  
  return padding_vec; 
}

std::vector<arm_navigation_msgs::LinkPadding> 
MechanismInterface::gripperPadding(std::string arm_name, double pad)
{
  std::vector<arm_navigation_msgs::LinkPadding> padding_vec;
  arm_navigation_msgs::LinkPadding padding;
  padding.padding = pad;  
  std::vector<std::string> links = handDescription().gripperTouchLinkNames(arm_name);
  for (size_t i=0; i<links.size(); i++)
  {
    padding.link_name = links[i];
    padding_vec.push_back(padding); 
  }  
  return padding_vec; 
}

} //namespace object_manipulator
