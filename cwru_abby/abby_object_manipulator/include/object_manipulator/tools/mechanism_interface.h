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
#ifndef _MECHANISM_INTERFACE_H_
#define _MECHANISM_INTERFACE_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Vector3.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <arm_navigation_msgs/MoveArmAction.h>

#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <arm_navigation_msgs/OrderedCollisionOperations.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <arm_navigation_msgs/ContactInformation.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <arm_navigation_msgs/GetRobotState.h>

#include <std_srvs/Empty.h>

#include <interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParams.h>

#include <arm_navigation_msgs/AttachedCollisionObject.h>

#include <planning_environment/models/collision_models.h>

#include <object_manipulation_msgs/ReactiveGraspAction.h>
#include <object_manipulation_msgs/ReactiveLiftAction.h>
#include <object_manipulation_msgs/ReactivePlaceAction.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

#include "object_manipulator/tools/exceptions.h"
#include "object_manipulator/tools/service_action_wrappers.h"


namespace object_manipulator {

//! A collection of ROS service and action clients needed for grasp execution
class MechanismInterface
{
 private:
  //! The root namespace node handle
  ros::NodeHandle root_nh_;

  //! The private namespace node handle
  ros::NodeHandle priv_nh_;

  //! Transform listener 
  tf::TransformListener listener_;

  //! Publisher for attached objects
  ros::Publisher attached_object_pub_;

  //! Name of the topic on which JointState messages come (for the current arm angles)
  std::string joint_states_topic_;

  //! Names for the arm controllers (only needed when used)
  //! Values are taken from the params arm_name_joint_controller and arm_name_cartesian_controller (for each arm_name)
  std::map<std::string, std::string> joint_controller_names_; 

  //! Names for the arm controllers (only needed when used)
  //! Values are taken from the params arm_name_joint_controller and arm_name_cartesian_controller (for each arm_name)
  std::map<std::string, std::string> cartesian_controller_names_; 

  //! Caches the last requested collision operations
  arm_navigation_msgs::OrderedCollisionOperations collision_operations_cache_;

  //! Caches the last requested link padding
  std::vector<arm_navigation_msgs::LinkPadding> link_padding_cache_;

  planning_environment::CollisionModels cm_;
  planning_models::KinematicState* planning_scene_state_;

  //! Keeps track of whether planning scene caching has been called at all
  bool planning_scene_cache_empty_;

  //! Used to disable planning scene caching altogether
  bool cache_planning_scene_;

  //! Sets the parameters for the interpolated IK server
  void setInterpolatedIKParams(std::string arm_name, int num_steps, 
			       int collision_check_resolution, bool start_from_end);

  //! Calls the switch_controllers service
  bool callSwitchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);
  
  //! Checks if the planning scene is the same as the last one requested, to avoid unnecessary calls
  /*! Returns true if the collision operations and link paddings are identical to the last ones requested. */
  bool cachePlanningScene(const arm_navigation_msgs::OrderedCollisionOperations& collision_operations,
                          const std::vector<arm_navigation_msgs::LinkPadding> &link_padding);

 public:

  //----------------------------- Service clients -------------------------------

  //! Client for IK information query that tells us the list of joints to work on
  MultiArmServiceWrapper<kinematics_msgs::GetKinematicSolverInfo> ik_query_client_;

  //! Client for the IK service
  MultiArmServiceWrapper<kinematics_msgs::GetConstraintAwarePositionIK> ik_service_client_;

  //! Client for the FK service
  MultiArmServiceWrapper<kinematics_msgs::GetPositionFK> fk_service_client_;

  //! Client for the Interpolated IK service
  MultiArmServiceWrapper<arm_navigation_msgs::GetMotionPlan> interpolated_ik_service_client_;  

  //! Client for setting the params of the Interpolated IK service
  MultiArmServiceWrapper<interpolated_ik_motion_planner::SetInterpolatedIKMotionPlanParams> 
    interpolated_ik_set_params_client_;

  //! Client for service that queries if a graps is currently active
  MultiArmServiceWrapper<object_manipulation_msgs::GraspStatus> grasp_status_client_;

  //! Client for service that checks state validity
  ServiceWrapper<arm_navigation_msgs::GetStateValidity> check_state_validity_client_;

  //! Client for the joint trajectory normalizer service
  ServiceWrapper<arm_navigation_msgs::FilterJointTrajectory> joint_trajectory_normalizer_service_;
  
  //! Client for the switch controller service
  ServiceWrapper<pr2_mechanism_msgs::SwitchController> switch_controller_service_;

  //! Client for the list controllers service
  ServiceWrapper<pr2_mechanism_msgs::ListControllers> list_controllers_service_;

  //! Client for service that gets robot state
  ServiceWrapper<arm_navigation_msgs::GetRobotState> get_robot_state_client_;

  //! Client for the service that gets the planning scene
  ServiceWrapper<arm_navigation_msgs::SetPlanningSceneDiff> set_planning_scene_diff_service_;

  //! Client for the service that resets the collision map
  ServiceWrapper<std_srvs::Empty> reset_collision_map_service_;

  //------------------------------ Action clients -------------------------------

  //! Action client for reactive grasping
  MultiArmActionWrapper<object_manipulation_msgs::ReactiveGraspAction> reactive_grasp_action_client_;

  //! Action client for reactive lifting
  MultiArmActionWrapper<object_manipulation_msgs::ReactiveLiftAction> reactive_lift_action_client_;

  //! Action client for reactive placing
  MultiArmActionWrapper<object_manipulation_msgs::ReactivePlaceAction> reactive_place_action_client_;

  //! Action client for move_arm
  MultiArmActionWrapper<arm_navigation_msgs::MoveArmAction> move_arm_action_client_;  

  //! Action client for executing a joint trajectory directly, without move arm
  MultiArmActionWrapper<pr2_controllers_msgs::JointTrajectoryAction> traj_action_client_;

  //! Action client for controlling the gripper for executing grasps
  MultiArmActionWrapper<object_manipulation_msgs::GraspHandPostureExecutionAction> hand_posture_client_;

  //! Action client for pointing the head at a target
  ActionWrapper<pr2_controllers_msgs::PointHeadAction> point_head_action_client_;

  //------------------------------ Publishers ----------------------------------

  //! Publisher for Cartesian pose commands
  MultiArmTopicWrapper<geometry_msgs::PoseStamped> cartesian_pub_;

  //! Publisher for Cartesian posture commands
  MultiArmTopicWrapper<std_msgs::Float64MultiArray> cartesian_posture_pub_;

  //! Publisher for changing Cartesian gains
  //MultiArmTopicWrapper<std_msgs::Float64MultiArray> cartesian_gains_pub_;
  //MultiArmTopicWrapper<std_msgs::CartesianGains> cartesian_gains_pub_;

  //------------------------------ Functionality -------------------------------

  //! Initializes all clients, then calls getIKInformation() and sets default interpolated IK params
  MechanismInterface();

  ~MechanismInterface() {
    if(planning_scene_state_ != NULL) {
      cm_.revertPlanningScene(planning_scene_state_);
    }
  }

  planning_environment::CollisionModels& getCollisionModels() {
    return cm_;
  }
  
  planning_models::KinematicState* getPlanningSceneState() const {
    return planning_scene_state_;
  }

  //------------- IK -------------

  //! Gets the current robot state
  void getRobotState(arm_navigation_msgs::RobotState& state);

  //! Sends the requsted collision operations and link padding to the environment server as a diff
  //! from the current planning scene on the server
  void getPlanningScene(const arm_navigation_msgs::OrderedCollisionOperations& collision_operations,
                        const std::vector<arm_navigation_msgs::LinkPadding> &link_padding);

  //! Checks if a given arm state is valid; joint_values must contain values for all joints of the arm
  bool checkStateValidity(std::string arm_name, const std::vector<double> &joint_values,
                          const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                          const std::vector<arm_navigation_msgs::LinkPadding> &link_padding);

  //! Checks if a given arm state is valid with no collision operations or link paddings
  bool checkStateValidity(std::string arm_name, const std::vector<double> &joint_values)
  {
    arm_navigation_msgs::OrderedCollisionOperations empty;
    std::vector<arm_navigation_msgs::LinkPadding> also_empty;
    return checkStateValidity(arm_name, joint_values, empty, also_empty);
  }

  //! Computes an IK solution for a desired pose
  bool getIKForPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
		    kinematics_msgs::GetConstraintAwarePositionIK::Response& ik_response,
                    const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                    const std::vector<arm_navigation_msgs::LinkPadding> &link_padding);

  //! Computes an FK solution
  bool getFK(std::string arm_name, 
	     std::vector<double> positions, 
	     geometry_msgs::PoseStamped &pose_stamped);


  //! Computes an interpolated IK path between two poses
  int getInterpolatedIK(std::string arm_name,
			geometry_msgs::PoseStamped start_pose,
			geometry_msgs::Vector3Stamped direction,
			float desired_trajectory_length,
			const std::vector<double> &seed_joint_position,
			const sensor_msgs::JointState &joint_state,
			const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
			const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
			bool reverse_trajectory,
			trajectory_msgs::JointTrajectory &trajectory,
			float &actual_trajectory_length);

  //------ traj controller  ------

  //! Uses the joint trajectory action to execute the desired trajectory
  void attemptTrajectory(std::string arm_name, const trajectory_msgs::JointTrajectory &trajectory, bool unnormalize);

  //! Convenience function that just gets the joint values for the trajectory, and assumes that 
  //! we are using the joint returned by the IK server in that order
  void attemptTrajectory(std::string arm_name, const std::vector< std::vector<double> > &positions,
                         bool unnormalize, float time_per_segment);

  //! Convenience function for assembly joint trajectory messages
  trajectory_msgs::JointTrajectory assembleJointTrajectory(std::string arm_name, 
			 const std::vector< std::vector<double> > &positions, 
			 float time_per_segment);

  //! Normalizes a trajectory (joint angle wrap-around)
  void unnormalizeTrajectory(std::string arm_name, const trajectory_msgs::JointTrajectory &input_trajectory,
			   trajectory_msgs::JointTrajectory &normalized_trajectory);

  //---------- move arm ----------

  //! Uses move arm to get to the desired set of joint values
  bool attemptMoveArmToGoal(std::string arm_name, const std::vector<double> &desired_joint_values,
                            const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                            const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
                            int max_tries = 5, bool reset_map_if_stuck = true, double timeout = 60);

  //! Modify a MoveArmGoal to exclude an area around the returned collision points
  void modifyMoveArmGoal(arm_navigation_msgs::MoveArmGoal &move_arm_goal, 
                         arm_navigation_msgs::ArmNavigationErrorCodes &error_code,
                         std::vector<arm_navigation_msgs::ContactInformation> &contact_info_);

  //! Call the move arm action client's cancelGoal function
  void cancelMoveArmGoal(std::string arm_name);

  //! Return the move arm action client's current state
  actionlib::SimpleClientGoalState getMoveArmState(std::string arm_name);

  //! Cancel any currently-running move_arm goals (returns false if timed out, otherwise true)
  bool clearMoveArmGoals(double timeout = 30);

  //! Uses move arm to a predefined arm position to the front, where the object can be transferred to the other arm
  bool moveArmToPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
                     const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                     const std::vector<arm_navigation_msgs::LinkPadding> &link_padding);

  //! Uses move arm to a desired pose while keeping the object in the hand level
  bool moveArmConstrained(std::string arm_name, const geometry_msgs::PoseStamped &commanded_pose,
                          const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                          const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
                          const arm_navigation_msgs::Constraints &path_constraints,
			  const double &redundancy = 0.0,
			  const bool &compute_viable_pose = true);

  //---------- gripper ----------

  //! Requests the hand to pre-grasp, grasp or release
  void handPostureGraspAction(std::string arm_name, const object_manipulation_msgs::Grasp &grasp, int goal, float max_contact_force);

  //! Queries the hand if a grasp is currently being correctly executed
  bool graspPostureQuery(std::string arm_name, const object_manipulation_msgs::Grasp grasp);

  //---------- head ----------

  //! Requests the head to point at a specific position
  bool pointHeadAction(const geometry_msgs::PointStamped &target, 
                       std::string pointing_frame="/narrow_stereo_optical_frame", bool wait_for_result = true);

  //------------- tf -------------

  //! Gets the current pose of the gripper in the frame_id specified in the gripper_pose.header
  geometry_msgs::PoseStamped getGripperPose(std::string arm_name, std::string frame_id);

  //! Computes the gripper pose transformed by a certain translation
  geometry_msgs::PoseStamped translateGripperPose(geometry_msgs::Vector3Stamped translation,
						  geometry_msgs::PoseStamped start_pose,
						  std::string arm_id);

  //! Transforms a pose from one frame to another; just passes through to the tf::Listener
  geometry_msgs::PoseStamped transformPose(const std::string target_frame, 
					   const geometry_msgs::PoseStamped &stamped_in);

  //! Transforms a cloud from one frame to another; just passes through to the tf::Listener
  void transformPointCloud(std::string target_frame, 
			   const sensor_msgs::PointCloud &cloud_in,
			   sensor_msgs::PointCloud &cloud_out);

  //! Given a grasp pose relative to the wrist roll link, returns the current pose of the grasped object
  geometry_msgs::PoseStamped getObjectPoseForGrasp(std::string arm_name, 
						   const geometry_msgs::Pose &grasp_pose);

  //! Converts all the internal components of a GraspableObject to the desired frame,
  //! and sets that frame as the reference_frame_id of that object.
  //! Does NOT convert the SceneRegion component which is camera frame by definition.
  void convertGraspableObjectComponentsToFrame(object_manipulation_msgs::GraspableObject &object,
                                               std::string frame_id);

  //! Just a convenience function
  float vectorLen(const geometry_msgs::Vector3 &vec)
  {
    tf::Vector3 v;
    tf::vector3MsgToTF(vec, v);
    return v.length();
  }

  //! Just a convenience function
  geometry_msgs::Vector3 normalize(const geometry_msgs::Vector3 &vec)
  {
    tf::Vector3 v;
    tf::vector3MsgToTF(vec, v);
    v.normalize();
    geometry_msgs::Vector3 vr;
    tf::vector3TFToMsg(v, vr);
    return vr;
  }

  geometry_msgs::Vector3 negate(const geometry_msgs::Vector3 &vec)
  {
    geometry_msgs::Vector3 v;
    v.x = - vec.x;
    v.y = - vec.y;
    v.z = - vec.z;
    return v;
  }

  //------------ controller switching ------------

  //! Check to see if a controller is running
  bool checkController(std::string controller);

  //! Switch one controller for another
  bool switchControllers(std::string start_controller, std::string stop_controller);

  //! Start one controller
  bool startController(std::string controller);

  //! Stop one controller
  bool stopController(std::string controller);

  //! Switch one arm from joint to Cartesian control
  bool switchToCartesian(std::string arm_name);

  //! Switch one arm from Cartesian to joint control
  bool switchToJoint(std::string arm_name);

  //! Get the name of the Cartesian controller for arm_name, returns an empty string if not found
  std::string cartesianControllerName(std::string arm_name);

  //! Get the name of the joint controller for arm_name, returns an empty string if not found
  std::string jointControllerName(std::string arm_name);

  //------------ Cartesian movement and related helper functions ------------

  //! Set a new desired PoseStamped for the Cartesian controller for arm_name 
  void sendCartesianPoseCommand(std::string arm_name, geometry_msgs::PoseStamped desired_pose);

  //! Set a new desired posture for the Cartesian controller for arm_name
  void sendCartesianPostureCommand(std::string arm_name, std::vector<double> arm_angles);

  /*
  //! Set new gains for the Cartesian controller for arm_name
  void sendCartesianGainsCommand(std::string arm_name, std::vector<double> gains, std::string frame_id, std::vector<double> fixed_frame);
  */

  //! Set the desired posture to the current arm angles
  void setCartesianPostureGoalToCurrentAngles(std::string arm_name);

  //! Ask the wrist to go incrementally towards a PoseStamped using the Cartesian controller until it gets there
  // (to within tolerances) or times out (returns 0 for error, 1 for got there, -1 for timed out)
  int moveArmToPoseCartesian(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
			      ros::Duration timeout, double dist_tol = 0.001, double angle_tol = 0.05,
                             double clip_dist = .02, double clip_angle = .16, 
			     double overshoot_dist = .005, double overshoot_angle = .087,
			     double timestep = 0.1,
                             const std::vector<double> &goal_posture_suggestion = std::vector<double>());

  //! Translate the gripper by direction using the Cartesian controllers
  int translateGripperCartesian(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
				ros::Duration timeout, double dist_tol = .001, double angle_tol = .05,
				double clip_dist = .02, double clip_angle = .16, 
				double overshoot_dist = 0.005, double overshoot_angle = 0.087, double timestep = 0.1);

  //! Euclidean distance/angle between two Pose messages
  void poseDists(geometry_msgs::Pose start, geometry_msgs::Pose end, double &pos_dist, double &angle);

  //! Euclidean distance/angle/axis/direction between two Eigen::Affine3ds
  void positionAndAngleDist(Eigen::Affine3d start, Eigen::Affine3d end, double &pos_dist, 
						double &angle, Eigen::Vector3d &axis, Eigen::Vector3d &direction);

  //! Clip a desired pose to be no more than clip_dist/clip_angle away from the given pose
  geometry_msgs::PoseStamped clipDesiredPose(const geometry_msgs::PoseStamped &current_pose,
                                                                 const geometry_msgs::PoseStamped &desired_pose,
                                                                 double clip_dist, double clip_angle,
                                                                 double &resulting_clip_fraction);

  //! Clip a desired pose to be no more than clip_dist/clip_angle away from the current gripper pose
  geometry_msgs::PoseStamped clipDesiredPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose, 
                                             double clip_dist, double clip_angle, double &resulting_clip_fraction);

  //! Clip a desired pose to be no more than clip_dist/clip_angle away from the current gripper pose
  // and clip an optional goal posture suggestion (arm angles) by the same amount
  geometry_msgs::PoseStamped clipDesiredPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose, 
                                             double clip_dist, double clip_angle, double &resulting_clip_fraction,
                                             const std::vector<double> &goal_posture_suggestion,
                                             std::vector<double> &clipped_posture_goal);

  //! Overshoot a desired pose by overshoot_dist and overshoot_angle
  geometry_msgs::PoseStamped overshootDesiredPose(std::string arm_name, 
					     const geometry_msgs::PoseStamped &desired_pose, 
					     double overshoot_dist, double overshoot_angle, 
					     double dist_tol, double angle_tol);

  //! Returns the joint names for the arm we are using
  std::vector<std::string> getJointNames(std::string arm_name);

  //! Get the current angles for arm_name
  bool getArmAngles(std::string arm_name, std::vector<double> &arm_angles);

  //------------ misc ------------

  //! Translates the gripper from its current pose to a new one using interpolated ik
  bool translateGripper(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
			arm_navigation_msgs::OrderedCollisionOperations ord, 
			const std::vector<arm_navigation_msgs::LinkPadding> &link_padding,
			float requested_distance, float min_distance,
			float &actual_distance);

  //! Returns the link padding vector for setting the desired padding to gripper fingertip links
  static std::vector<arm_navigation_msgs::LinkPadding> fingertipPadding(std::string arm_name, double pad);

  //! Returns the link padding vector for setting the desired padding to gripper touch links
  static std::vector<arm_navigation_msgs::LinkPadding> gripperPadding(std::string arm_name, double pad);

  //------------ attached objects (collision space) ------------

  //! Attaches a given object to the gripper
  void attachObjectToGripper(std::string arm_name, std::string object_collision_name);

  //! Detached the indicated object from the gripper and adds it back in as a non-attached object
  void detachAndAddBackObjectsAttachedToGripper(std::string arm_name, std::string object_collision_name);

  //! Detach and remove all objects attached to the gripper
  void detachAllObjectsFromGripper(std::string arm_name);
};

//! Returns a MechanismInterface singleton
/*! Due to problems with waitForServer, it is not recommended to use multiple instances
  of the MechanismInterface in the same node. This function provides a singleton.

  CAREFUL WITH MULTI-THREADED CODE!!!
*/
inline MechanismInterface& mechInterface()
{
  static MechanismInterface mech_interface;
  return mech_interface;
}

} //namespace object_manipulator

#endif
