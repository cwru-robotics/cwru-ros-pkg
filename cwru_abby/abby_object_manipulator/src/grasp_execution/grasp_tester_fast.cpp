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

// Author(s): E. Gil Jones

#include <sstream>

#include "object_manipulator/grasp_execution/grasp_tester_fast.h"

#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"
#include "object_manipulator/tools/mechanism_interface.h"

//#include <demo_synchronizer/synchronizer_client.h>

using object_manipulation_msgs::GraspResult;
using arm_navigation_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

    void  print_contacts(planning_environment::CollisionModels* cm, planning_models::KinematicState* state) {
        std::vector<arm_navigation_msgs::ContactInformation> contacts;
        cm->getAllCollisionsForState(*state, contacts,1);
        if(contacts.size() == 0) {
            ROS_WARN_STREAM("Collision reported but no contacts");
        }
        std::vector<std::string> names;
        for(unsigned int j = 0; j < contacts.size(); j++) {
            names.push_back(contacts[j].contact_body_1);

            ROS_DEBUG_STREAM_NAMED("manipulation", "Collision between " << contacts[j].contact_body_1
                             << " and " << contacts[j].contact_body_2);
        }
    }

    void visualize_grasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,
                          const std::vector<GraspExecutionInfo> &execution_info,
                          ros::Publisher &vis_marker_publisher) {
        /* display markers for all of the grasps */
        for(unsigned int i = 0; i < grasps.size(); i++)
        {
            float r, g, b;
            switch(execution_info[i].result_.result_code)
            {
            case GraspResult::GRASP_OUT_OF_REACH:
                r = 1.0; g = 0.0; b = 0.0;
                break;
            case GraspResult::GRASP_IN_COLLISION:
                r = 1.0; g = 0.0; b = 0.0;
                break;
            case GraspResult::GRASP_UNFEASIBLE:
                r = 1.0; g = 0.0; b = 0.0;
                break;
            case GraspResult::PREGRASP_OUT_OF_REACH:
                r = 1.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::PREGRASP_IN_COLLISION:
                r = 1.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::PREGRASP_UNFEASIBLE:
                r = 1.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::LIFT_OUT_OF_REACH:
                r = 0.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::LIFT_IN_COLLISION:
                r = 0.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::LIFT_UNFEASIBLE:
                r = 0.0; g = 0.0; b = 1.0;
                break;
            case GraspResult::SUCCESS:
                r = 0.0; g = 1.0; b = 0.0;
                break;
            default:
                r = 0.0; g = 0.0; b = 0.0;
            }

            visualization_msgs::Marker marker;

            marker.pose = grasps[i].grasp_pose;
            marker.header.frame_id = pickup_goal.target.reference_frame_id;
            marker.header.stamp = ros::Time::now();
            std::ostringstream marker_ns;
            marker_ns << "grasp " << i << " (" << execution_info[i].result_.result_code << ")";
            marker.ns = marker_ns.str();

            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration();

            marker.type = visualization_msgs::Marker::ARROW;
            marker.scale.x = 0.1235;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = 1.0;

            marker.id = 0;

            ROS_DEBUG("Displaying marker %d with ns %s", i, marker.ns.c_str());
            vis_marker_publisher.publish(marker);
        }
    }

    GraspTesterFast::GraspTesterFast(planning_environment::CollisionModels* cm,
                                     const std::string& plugin_name)
                                         : GraspTester(),
                                         consistent_angle_(M_PI/12.0),
                                         num_points_(10),
                                         redundancy_(2),
                                         cm_(cm),
                                         state_(NULL),
                                         kinematics_loader_("kinematics_base","kinematics::KinematicsBase")
    {
        ros::NodeHandle nh;

        vis_marker_publisher_ = nh.advertise<visualization_msgs::Marker> ("grasp_executor_fast", 128);
        vis_marker_array_publisher_ = nh.advertise<visualization_msgs::MarkerArray> ("grasp_executor_fast_array", 128);

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

    GraspTesterFast::~GraspTesterFast()
    {
        for(std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*>::iterator it = ik_solver_map_.begin();
        it != ik_solver_map_.end();
        it++) {
            delete it->second;
        }
    }

    planning_environment::CollisionModels* GraspTesterFast::getCollisionModels() {
        if(cm_ == NULL) {
            return &mechInterface().getCollisionModels();
        } else {
            return cm_;
        }
    }

    planning_models::KinematicState* GraspTesterFast::getPlanningSceneState() {
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

    /*! Zero padding on fingertip links */
    std::vector<arm_navigation_msgs::LinkPadding>
            GraspTesterFast::linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal)
    {
        return concat(MechanismInterface::gripperPadding(pickup_goal.arm_name, 0.0),
                      pickup_goal.additional_link_padding);
    }



    void GraspTesterFast::getGroupLinks(const std::string& group_name,
                                        std::vector<std::string>& group_links)
    {
        group_links.clear();
        const planning_models::KinematicModel::JointModelGroup* jmg =
                getCollisionModels()->getKinematicModel()->getModelGroup(group_name);
        if(jmg == NULL) return;
        group_links = jmg->getGroupLinkNames();
    }

    void GraspTesterFast::getGroupJoints(const std::string& group_name,
                                         std::vector<std::string>& group_joints)
    {
        if(ik_solver_map_.find(group_name) == ik_solver_map_.end()) {
            ROS_ERROR_STREAM("No group for solver " << group_name);
            return;
        }
        group_joints = ik_solver_map_[group_name]->getJointNames();
    }

    bool GraspTesterFast::getInterpolatedIK(const std::string& arm_name,
                                            const tf::Transform& first_pose,
                                            const tf::Vector3& direction,
                                            const double& distance,
                                            const std::vector<double>& ik_solution,
                                            const bool& reverse,
                                            const bool& premultiply,
                                            trajectory_msgs::JointTrajectory& traj) {

        std::map<std::string, double> ik_solution_map;
        for(unsigned int i = 0; i < traj.joint_names.size(); i++) {
            ik_solution_map[traj.joint_names[i]] = ik_solution[i];
        }

        getPlanningSceneState()->setKinematicState(ik_solution_map);

        geometry_msgs::Pose start_pose;
        tf::poseTFToMsg(first_pose, start_pose);

        arm_navigation_msgs::Constraints emp;
        arm_navigation_msgs::ArmNavigationErrorCodes error_code;
        return ik_solver_map_[arm_name]->interpolateIKDirectional(start_pose,
                                                                  direction,
                                                                  distance,
                                                                  emp,
                                                                  getPlanningSceneState(),
                                                                  error_code,
                                                                  traj,
                                                                  redundancy_,
                                                                  consistent_angle_,
                                                                  reverse,
                                                                  premultiply,
                                                                  num_points_,
                                                                  ros::Duration(2.5),
                                                                  false);
    }

    /* this doesn't get called since testGrasps() does everything, but we still need to have
   this function since it is pure virtual in the base class */
    void GraspTesterFast::testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                    const object_manipulation_msgs::Grasp &grasp,
                                    GraspExecutionInfo &execution_info)  {}

    void GraspTesterFast::testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                                     const std::vector<object_manipulation_msgs::Grasp> &grasps,
                                     std::vector<GraspExecutionInfo> &execution_info,
                                     bool return_on_first_hit)

    {
        ros::WallTime start = ros::WallTime::now();
        std::map<unsigned int, unsigned int> outcome_count;
        planning_environment::CollisionModels* cm = getCollisionModels();
        planning_models::KinematicState* state = getPlanningSceneState();
        std::map<std::string, double> planning_scene_state_values;
        state->getKinematicStateValues(planning_scene_state_values);

        std::vector<std::string> end_effector_links, arm_links;
        end_effector_links = handDescription().gripperTouchLinkNames(pickup_goal.arm_name);
        //getGroupLinks(handDescription().gripperCollisionName(pickup_goal.arm_name), end_effector_links);
        getGroupLinks(handDescription().armGroup(pickup_goal.arm_name), arm_links);

        collision_space::EnvironmentModel::AllowedCollisionMatrix original_acm = cm->getCurrentAllowedCollisionMatrix();
        cm->disableCollisionsForNonUpdatedLinks(pickup_goal.arm_name); /* disable collisions for all links not in the arm we are using */
        collision_space::EnvironmentModel::AllowedCollisionMatrix group_disable_acm = cm->getCurrentAllowedCollisionMatrix();
        collision_space::EnvironmentModel::AllowedCollisionMatrix object_disable_acm = group_disable_acm;
        object_disable_acm.changeEntry(pickup_goal.collision_object_name, end_effector_links, true);
        collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_disable_acm = object_disable_acm;
        if(pickup_goal.allow_gripper_support_collision)
        {
            ROS_DEBUG("Disabling collisions between gripper and support surface");
            if(pickup_goal.collision_support_surface_name == "\"all\"")
            {
                for(unsigned int i = 0; i < end_effector_links.size(); i++){
                    object_support_disable_acm.changeEntry(end_effector_links[i], true);
                }
            }
            else{
                ROS_DEBUG("not all");
                object_support_disable_acm.changeEntry(pickup_goal.collision_support_surface_name, end_effector_links, true);
            }
        }

        /* allows collisions between the object and the and effector */
        collision_space::EnvironmentModel::AllowedCollisionMatrix object_all_arm_disable_acm = object_disable_acm;

        /* allows collisions between the end effector and the object, as well as between the end effector and the support surface */
        collision_space::EnvironmentModel::AllowedCollisionMatrix object_support_all_arm_disable_acm = object_support_disable_acm;

        /* allows collisions between the arm and anything else */
        collision_space::EnvironmentModel::AllowedCollisionMatrix group_all_arm_disable_acm = group_disable_acm;

        //turning off collisions for the arm associated with this end effector
        for(unsigned int i = 0; i < arm_links.size(); i++) {
            object_all_arm_disable_acm.changeEntry(arm_links[i], true);
            object_support_all_arm_disable_acm.changeEntry(arm_links[i], true);
            group_all_arm_disable_acm.changeEntry(arm_links[i], true);
        }
        cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);

        //first we apply link padding for grasp check
        cm->applyLinkPaddingToCollisionSpace(linkPaddingForGrasp(pickup_goal));

        //setup that's not grasp specific
        std_msgs::Header target_header;
        target_header.frame_id = pickup_goal.target.reference_frame_id;

        bool in_object_frame = false;
        tf::Transform obj_pose(tf::Quaternion(0,0,0,1.0), tf::Vector3(0.0,0.0,0.0));
        if(pickup_goal.target.reference_frame_id == pickup_goal.collision_object_name) {
            in_object_frame = true;
            geometry_msgs::PoseStamped obj_world_pose_stamped;
            cm->convertPoseGivenWorldTransform(*state,
                                               cm->getWorldFrameId(),
                                               pickup_goal.target.potential_models[0].pose.header,
                                               pickup_goal.target.potential_models[0].pose.pose,
                                               obj_world_pose_stamped);
            tf::poseMsgToTF(obj_world_pose_stamped.pose, obj_pose);
        }

        execution_info.clear();
        execution_info.resize(grasps.size());

        tf::Vector3 pregrasp_dir;
        tf::vector3MsgToTF(doNegate(handDescription().approachDirection(pickup_goal.arm_name)), pregrasp_dir);
        pregrasp_dir.normalize();

        tf::Vector3 lift_dir;
        tf::vector3MsgToTF(pickup_goal.lift.direction.vector, lift_dir);
        lift_dir.normalize();
        tf::Vector3 distance_lift_dir = lift_dir*fabs(pickup_goal.lift.desired_distance);
        tf::Transform lift_trans(tf::Quaternion(0,0,0,1.0), distance_lift_dir);

        std::vector<tf::Transform> grasp_poses(grasps.size());

        ros::Rate debug_rate(0.2);

        //now this is grasp specific
        for(unsigned int i = 0; i < grasps.size(); i++) {

            //check whether the grasp pose is ok (only checking hand, not arms)
            //using pre-grasp posture, cause grasp_posture only matters for closing the gripper
            std::map<std::string, double> pre_grasp_joint_vals;
            for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                pre_grasp_joint_vals[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
            }
            state->setKinematicState(pre_grasp_joint_vals);

            //always true
            execution_info[i].result_.continuation_possible = true;

            if(!in_object_frame) {
                geometry_msgs::PoseStamped grasp_world_pose_stamped;
                if(!cm->convertPoseGivenWorldTransform(*state,
                                                       cm->getWorldFrameId(),
                                                       target_header,
                                                       grasps[i].grasp_pose,
                                                       grasp_world_pose_stamped)) {
                    ROS_WARN_STREAM("Can't convert into non-object frame " << target_header.frame_id);
                    continue;
                }
                tf::poseMsgToTF(grasp_world_pose_stamped.pose, grasp_poses[i]);
            } else {
                tf::Transform gp;
                tf::poseMsgToTF(grasps[i].grasp_pose, gp);
                grasp_poses[i] = obj_pose*gp;
            }
            state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(pickup_goal.arm_name),grasp_poses[i]);

            if(cm->isKinematicStateInCollision(*state)) {
                ROS_DEBUG_STREAM("Grasp in collision");
                print_contacts(cm, state);

                std_msgs::ColorRGBA col_pregrasp;
                col_pregrasp.r = 0.0;
                col_pregrasp.g = 1.0;
                col_pregrasp.b = 1.0;
                col_pregrasp.a = 1.0;
                visualization_msgs::MarkerArray arr;
                cm->getRobotMarkersGivenState(*state, arr, col_pregrasp,
                                              "grasp_in_collision",
                                              ros::Duration(0.0),
                                              &end_effector_links);
                vis_marker_array_publisher_.publish(arr);
                execution_info[i].result_.result_code = GraspResult::GRASP_IN_COLLISION;
                outcome_count[GraspResult::GRASP_IN_COLLISION]++;
            } else {
                execution_info[i].result_.result_code = 0;
            }
        }

        //now we revert link paddings for pre-grasp and lift checks
        cm->revertCollisionSpacePaddingToDefault();

        //first we do lift, with the hand in the grasp posture (collisions allowed between gripper and object)
        cm->setAlteredAllowedCollisionMatrix(object_support_all_arm_disable_acm);
        for(unsigned int i = 0; i < grasps.size(); i++) {

            if(execution_info[i].result_.result_code != 0) continue;

            std::map<std::string, double> grasp_joint_vals;
            for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
                grasp_joint_vals[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
            }
            state->setKinematicState(grasp_joint_vals);

            tf::Transform lift_pose = lift_trans*grasp_poses[i];
            state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(pickup_goal.arm_name),lift_pose);

            if(cm->isKinematicStateInCollision(*state)) {
                ROS_DEBUG_STREAM_NAMED("manipulation", "Lift in collision");
                print_contacts(cm, state);
                execution_info[i].result_.result_code = GraspResult::LIFT_IN_COLLISION;
                outcome_count[GraspResult::LIFT_IN_COLLISION]++;
            }
        }

        //now we do pre-grasp not allowing object touch, but with arms disabled
        cm->setAlteredAllowedCollisionMatrix(group_all_arm_disable_acm);

        for(unsigned int i = 0; i < grasps.size(); i++) {

            if(execution_info[i].result_.result_code != 0) continue;

            //opening the gripper back to pre_grasp
            std::map<std::string, double> pre_grasp_joint_vals;
            for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                pre_grasp_joint_vals[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
            }
            state->setKinematicState(pre_grasp_joint_vals);

            tf::Vector3 distance_pregrasp_dir = pregrasp_dir * fabs(grasps[0].desired_approach_distance);
            tf::Transform pre_grasp_trans(tf::Quaternion(0,0,0,1.0), distance_pregrasp_dir);
            tf::Transform pre_grasp_pose = grasp_poses[i]*pre_grasp_trans;
            state->updateKinematicStateWithLinkAt(handDescription().gripperFrame(pickup_goal.arm_name),pre_grasp_pose);

            if(cm->isKinematicStateInCollision(*state)) {
                ROS_DEBUG_STREAM_NAMED("manipulation", "Pre-grasp in collision");
                print_contacts(cm, state);

                std_msgs::ColorRGBA col_pregrasp;
                col_pregrasp.r = 1.0;
                col_pregrasp.g = 0.0;
                col_pregrasp.b = 1.0;
                col_pregrasp.a = 1.0;
                visualization_msgs::MarkerArray arr;
                cm->getRobotMarkersGivenState(*state, arr, col_pregrasp,
                                              "pre_grasp_in_collision",
                                              ros::Duration(0.0),
                                              &end_effector_links);
                vis_marker_array_publisher_.publish(arr);

                execution_info[i].result_.result_code = GraspResult::PREGRASP_IN_COLLISION;
                outcome_count[GraspResult::PREGRASP_IN_COLLISION]++;
                continue;
            }
        }

        visualize_grasps(pickup_goal, grasps, execution_info, vis_marker_publisher_);

        std_msgs::Header world_header;
        world_header.frame_id = cm->getWorldFrameId();
        const std::vector<std::string>& joint_names = ik_solver_map_[pickup_goal.arm_name]->getJointNames();

        if(return_on_first_hit) {

            bool last_ik_failed = false;
            for(unsigned int i = 0; i < grasps.size(); i++) {

                if(execution_info[i].result_.result_code != 0) continue;

                if(!last_ik_failed) {
                    //now we move to the ik portion, which requires re-enabling collisions for the arms
                    cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

                    //and also reducing link paddings
                    cm->applyLinkPaddingToCollisionSpace(linkPaddingForGrasp(pickup_goal));
                }
                //getting back to original state for seed
                state->setKinematicState(planning_scene_state_values);

                //adjusting planning scene state for pre-grasp
                std::map<std::string, double> pre_grasp_values;
                for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                    pre_grasp_values[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
                }
                state->setKinematicState(pre_grasp_values);

                //now call ik for grasp
                geometry_msgs::Pose grasp_geom_pose;
                tf::poseTFToMsg(grasp_poses[i], grasp_geom_pose);
                geometry_msgs::PoseStamped base_link_grasp_pose;
                cm->convertPoseGivenWorldTransform(*state,
                                                   ik_solver_map_[pickup_goal.arm_name]->getBaseName(),
                                                   world_header,
                                                   grasp_geom_pose,
                                                   base_link_grasp_pose);

                arm_navigation_msgs::Constraints emp;
                sensor_msgs::JointState solution;
                arm_navigation_msgs::ArmNavigationErrorCodes error_code;
                ROS_DEBUG_STREAM("X y z " << base_link_grasp_pose.pose.position.x << " "
                                << base_link_grasp_pose.pose.position.y << " "
                                << base_link_grasp_pose.pose.position.z);
                if(!ik_solver_map_[pickup_goal.arm_name]->findConstraintAwareSolution(base_link_grasp_pose.pose,
                                                                                      emp,
                                                                                      state,
                                                                                      solution,
                                                                                      error_code,
                                                                                      false)) {
                    ROS_DEBUG_STREAM_NAMED("manipulation", "Grasp out of reach");
                    std_msgs::ColorRGBA col_pregrasp;
                    col_pregrasp.r = 0.0;
                    col_pregrasp.g = 1.0;
                    col_pregrasp.b = 1.0;
                    col_pregrasp.a = 1.0;
                    visualization_msgs::MarkerArray arr;
                    cm->getRobotMarkersGivenState(*state, arr, col_pregrasp,
                                                  "out_of_reach",
                                                  ros::Duration(0.0),
                                                  &end_effector_links);
                    vis_marker_array_publisher_.publish(arr);

                    execution_info[i].result_.result_code = GraspResult::GRASP_OUT_OF_REACH;
                    outcome_count[GraspResult::GRASP_OUT_OF_REACH]++;
                    last_ik_failed = true;
                    continue;
                } else {
                    last_ik_failed = false;
                }

                std::map<std::string, double> ik_map_pre_grasp;
                std::map<std::string, double> ik_map_grasp;
                for(unsigned int j = 0; j < joint_names.size(); j++) {
                    ik_map_pre_grasp[joint_names[j]] = solution.position[j];
                }
                ik_map_grasp = ik_map_pre_grasp;

                for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                    ik_map_pre_grasp[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
                }
                for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
                    ik_map_grasp[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
                }

                state->setKinematicState(ik_map_pre_grasp);

                //now we solve interpolated ik
                tf::Transform base_link_bullet_grasp_pose;
                tf::poseMsgToTF(base_link_grasp_pose.pose, base_link_bullet_grasp_pose);
                execution_info[i].approach_trajectory_.joint_names = joint_names;
                //now we need to do interpolated ik
                if(!getInterpolatedIK(pickup_goal.arm_name,
                                      base_link_bullet_grasp_pose,
                                      pregrasp_dir,
                                      grasps[i].desired_approach_distance,
                                      solution.position,
                                      true,
                                      false,
                                      execution_info[i].approach_trajectory_)) {
                    ROS_DEBUG_STREAM_NAMED("manipulation", "No interpolated IK for pre-grasp to grasp");
                    execution_info[i].result_.result_code = GraspResult::PREGRASP_UNFEASIBLE;
                    outcome_count[GraspResult::PREGRASP_UNFEASIBLE]++;
                    continue;
                }

                ROS_DEBUG_STREAM_NAMED("manipulation", "Last approach point is " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[0] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[1] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[2] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[3] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[4] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[5] << " " <<
                                 execution_info[i].approach_trajectory_.points.back().positions[6]);

                state->setKinematicState(ik_map_grasp);
                execution_info[i].lift_trajectory_.joint_names = joint_names;
                if(!getInterpolatedIK(pickup_goal.arm_name,
                                      base_link_bullet_grasp_pose,
                                      lift_dir,
                                      pickup_goal.lift.desired_distance,
                                      solution.position,
                                      false,
                                      true,
                                      execution_info[i].lift_trajectory_)) {
                    ROS_DEBUG_STREAM_NAMED("manipulation", "No interpolated IK for grasp to lift");
                    execution_info[i].result_.result_code = GraspResult::LIFT_UNFEASIBLE;
                    outcome_count[GraspResult::LIFT_UNFEASIBLE]++;
                    continue;
                }

                ROS_DEBUG_STREAM_NAMED("manipulation", "First lift point is " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[0] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[1] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[2] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[3] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[4] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[5] << " " <<
                                 execution_info[i].lift_trajectory_.points.front().positions[6]);

                //now we revert link paddings and object collisions and do a final check for the initial ik points
                cm->revertCollisionSpacePaddingToDefault();

                //the start of the approach needs to be collision-free according to the default collision matrix
                cm->setAlteredAllowedCollisionMatrix(group_disable_acm);

                if(execution_info[i].approach_trajectory_.points.empty()) {
                    ROS_WARN_STREAM("No result code and no points in approach trajectory");
                    continue;
                }

                std::map<std::string, double> pre_grasp_ik = ik_map_pre_grasp;
                for(unsigned int j = 0; j < joint_names.size(); j++) {
                    pre_grasp_ik[joint_names[j]] = execution_info[i].approach_trajectory_.points[0].positions[j];
                }
                state->setKinematicState(pre_grasp_ik);
                if(cm->isKinematicStateInCollision(*state)) {
                    ROS_DEBUG_STREAM_NAMED("manipulation", "Final pre-grasp check failed");
                    std::vector<arm_navigation_msgs::ContactInformation> contacts;
                    execution_info[i].result_.result_code = GraspResult::PREGRASP_OUT_OF_REACH;
                    outcome_count[GraspResult::PREGRASP_OUT_OF_REACH]++;
                    continue;
                }

                //the object will be attached to the gripper for the lift, so we don't care if the object collides with the hand
                cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

                if(execution_info[i].lift_trajectory_.points.empty()) {
                    ROS_WARN_STREAM("No result code and no points in lift trajectory");
                    continue;
                }
                std::map<std::string, double> lift_ik = ik_map_pre_grasp;
                for(unsigned int j = 0; j < joint_names.size(); j++) {
                    lift_ik[joint_names[j]] = execution_info[i].lift_trajectory_.points.back().positions[j];
                }
                state->setKinematicState(lift_ik);
                if(cm->isKinematicStateInCollision(*state)) {
                    ROS_DEBUG_STREAM_NAMED("manipulation", "Final lift check failed");
                    execution_info[i].result_.result_code = GraspResult::LIFT_OUT_OF_REACH;
                    outcome_count[GraspResult::LIFT_OUT_OF_REACH]++;
                    continue;
                } else {
                    ROS_DEBUG_STREAM("Everything successful");
                    execution_info[i].result_.result_code = GraspResult::SUCCESS;
                    execution_info.resize(i+1);
                    outcome_count[GraspResult::SUCCESS]++;
                    break;
                }
            }
            for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
            it != outcome_count.end();
            it++) {
                ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
            }
            cm->setAlteredAllowedCollisionMatrix(original_acm);
            ROS_INFO_STREAM("Took " << (ros::WallTime::now()-start).toSec());
            return;
        }


        //now we move to the ik portion, which requires re-enabling collisions for the arms
        cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

        //and also reducing link paddings
        cm->applyLinkPaddingToCollisionSpace(linkPaddingForGrasp(pickup_goal));

        for(unsigned int i = 0; i < grasps.size(); i++) {

            if(execution_info[i].result_.result_code != 0) continue;

            //getting back to original state for seed
            state->setKinematicState(planning_scene_state_values);

            //adjusting planning scene state for pre-grasp
            std::map<std::string, double> pre_grasp_values;
            for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                pre_grasp_values[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
            }
            state->setKinematicState(pre_grasp_values);

            //now call ik for grasp
            geometry_msgs::Pose grasp_geom_pose;
            tf::poseTFToMsg(grasp_poses[i], grasp_geom_pose);
            geometry_msgs::PoseStamped base_link_grasp_pose;
            cm->convertPoseGivenWorldTransform(*state,
                                               ik_solver_map_[pickup_goal.arm_name]->getBaseName(),
                                               world_header,
                                               grasp_geom_pose,
                                               base_link_grasp_pose);

            arm_navigation_msgs::Constraints emp;
            sensor_msgs::JointState solution;
            arm_navigation_msgs::ArmNavigationErrorCodes error_code;
            if(!ik_solver_map_[pickup_goal.arm_name]->findConstraintAwareSolution(base_link_grasp_pose.pose,
                                                                                  emp,
                                                                                  state,
                                                                                  solution,
                                                                                  error_code,
                                                                                  false)) {
                ROS_DEBUG_STREAM_NAMED("manipulation", "Grasp out of reach");
                print_contacts(cm, state);
                execution_info[i].result_.result_code = GraspResult::GRASP_OUT_OF_REACH;
                outcome_count[GraspResult::GRASP_OUT_OF_REACH]++;
                continue;
            }

            std::map<std::string, double> ik_map_pre_grasp;
            std::map<std::string, double> ik_map_grasp;
            for(unsigned int j = 0; j < joint_names.size(); j++) {
                ik_map_pre_grasp[joint_names[j]] = solution.position[j];
            }
            ik_map_grasp = ik_map_pre_grasp;

            for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                ik_map_pre_grasp[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
            }
            for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
                ik_map_grasp[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
            }

            state->setKinematicState(ik_map_pre_grasp);

            //now we solve interpolated ik
            tf::Transform base_link_bullet_grasp_pose;
            tf::poseMsgToTF(base_link_grasp_pose.pose, base_link_bullet_grasp_pose);

            /* try to do interpolated IK from grasp back to pregrasp */
            execution_info[i].approach_trajectory_.joint_names = joint_names;
            if(!getInterpolatedIK(pickup_goal.arm_name,
                                  base_link_bullet_grasp_pose,
                                  pregrasp_dir,
                                  grasps[i].desired_approach_distance,
                                  solution.position,
                                  true,
                                  false,
                                  execution_info[i].approach_trajectory_)) {
                ROS_DEBUG_STREAM_NAMED("manipulation", "No interpolated IK for pre-grasp to grasp");
                execution_info[i].result_.result_code = GraspResult::PREGRASP_UNFEASIBLE;
                outcome_count[GraspResult::PREGRASP_UNFEASIBLE]++;
                continue;
            }

            /* try to do interpolated IK for the lift */
            state->setKinematicState(ik_map_grasp);
            execution_info[i].lift_trajectory_.joint_names = joint_names;
            if(!getInterpolatedIK(pickup_goal.arm_name,
                                  base_link_bullet_grasp_pose,
                                  lift_dir,
                                  pickup_goal.lift.desired_distance,
                                  solution.position,
                                  false,
                                  true,
                                  execution_info[i].lift_trajectory_)) {
                ROS_DEBUG_STREAM_NAMED("manipulation","No interpolated IK for grasp to lift");
                execution_info[i].result_.result_code = GraspResult::LIFT_UNFEASIBLE;
                outcome_count[GraspResult::LIFT_UNFEASIBLE]++;
                continue;
            }
        }

        //now we revert link paddings and object collisions and do a final check for the initial ik points
        cm->revertCollisionSpacePaddingToDefault();

        cm->setAlteredAllowedCollisionMatrix(group_disable_acm);

        for(unsigned int i = 0; i < grasps.size(); i++) {

            if(execution_info[i].result_.result_code != 0) continue;

            if(execution_info[i].approach_trajectory_.points.empty()) {
                ROS_WARN_STREAM("No result code and no points in approach trajectory");
                continue;
            }

            std::map<std::string, double> ik_map_pre_grasp;
            for(unsigned int j = 0; j < joint_names.size(); j++) {
                ik_map_pre_grasp[joint_names[j]] = execution_info[i].approach_trajectory_.points[0].positions[j];
            }
            for(unsigned int j = 0; j < grasps[i].pre_grasp_posture.name.size(); j++) {
                ik_map_pre_grasp[grasps[i].pre_grasp_posture.name[j]] = grasps[i].pre_grasp_posture.position[j];
            }
            state->setKinematicState(ik_map_pre_grasp);
            if(cm->isKinematicStateInCollision(*state)) {
                ROS_DEBUG_STREAM_NAMED("manipulation","Final pre-grasp check failed");
                print_contacts(cm, state);

                std::vector<arm_navigation_msgs::ContactInformation> contacts;
                cm->getAllCollisionsForState(*state, contacts,1);
                std::vector<std::string> names;
                for(unsigned int j = 0; j < contacts.size(); j++) {
                    names.push_back(contacts[j].contact_body_1);
                }

                std_msgs::ColorRGBA col_pregrasp;
                col_pregrasp.r = 0.0;
                col_pregrasp.g = 0.0;
                col_pregrasp.b = 1.0;
                col_pregrasp.a = 1.0;
                visualization_msgs::MarkerArray arr;
                cm->getRobotPaddedMarkersGivenState(*state, arr, col_pregrasp,
                                                    "padded",
                                                    ros::Duration(0.0),
                                                    &names);
                vis_marker_array_publisher_.publish(arr);
                execution_info[i].result_.result_code = GraspResult::PREGRASP_OUT_OF_REACH;
                outcome_count[GraspResult::PREGRASP_OUT_OF_REACH]++;
                continue;
            }

        }

        //now we need to disable collisions with the object for lift
        cm->setAlteredAllowedCollisionMatrix(object_support_disable_acm);

        for(unsigned int i = 0; i < grasps.size(); i++) {

            if(execution_info[i].result_.result_code != 0) continue;

            if(execution_info[i].lift_trajectory_.points.empty()) {
                ROS_WARN_STREAM("No result code and no points in lift trajectory");
                continue;
            }
            std::map<std::string, double> ik_map_grasp;
            for(unsigned int j = 0; j < joint_names.size(); j++) {
                ik_map_grasp[joint_names[j]] = execution_info[i].lift_trajectory_.points.back().positions[j];
            }
            for(unsigned int j = 0; j < grasps[i].grasp_posture.name.size(); j++) {
                ik_map_grasp[grasps[i].grasp_posture.name[j]] = grasps[i].grasp_posture.position[j];
            }
            state->setKinematicState(ik_map_grasp);
            if(cm->isKinematicStateInCollision(*state)) {
                ROS_DEBUG_STREAM_NAMED("manipulation","Final lift check failed");
                print_contacts(cm, state);
                execution_info[i].result_.result_code = GraspResult::LIFT_OUT_OF_REACH;
                outcome_count[GraspResult::LIFT_OUT_OF_REACH]++;
                continue;
            } else {
                ROS_DEBUG_STREAM_NAMED("manipulation", "Everything successful");
                execution_info[i].result_.result_code = GraspResult::SUCCESS;
                outcome_count[GraspResult::SUCCESS]++;
            }
        }
        cm->setAlteredAllowedCollisionMatrix(original_acm);

        visualize_grasps(pickup_goal, grasps, execution_info, vis_marker_publisher_);

        ROS_DEBUG_STREAM("Took " << (ros::WallTime::now()-start).toSec());

        for(std::map<unsigned int, unsigned int>::iterator it = outcome_count.begin();
        it != outcome_count.end();
        it++) {
            ROS_INFO_STREAM("Outcome " << it->first << " count " << it->second);
        }
    }


} //namespace object_manipulator
