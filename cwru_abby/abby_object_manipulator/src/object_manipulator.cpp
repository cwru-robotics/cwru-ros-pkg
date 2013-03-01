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

#include "object_manipulator/object_manipulator.h"

#include <algorithm>

//#define PROF_ENABLED
//#include <profiling/profiling.h>
//PROF_DECLARE(TOTAL_PICKUP_TIMER);

//#include <demo_synchronizer/synchronizer_client.h>

#include <object_manipulation_msgs/tools.h>

//old style executors
#include "object_manipulator/grasp_execution/grasp_executor_with_approach.h"
#include "object_manipulator/grasp_execution/reactive_grasp_executor.h"
#include "object_manipulator/grasp_execution/unsafe_grasp_executor.h"
#include "object_manipulator/place_execution/place_executor.h"

//new style executors
#include "object_manipulator/grasp_execution/approach_lift_grasp.h"
#include "object_manipulator/grasp_execution/grasp_tester_fast.h"
#include "object_manipulator/place_execution/descend_retreat_place.h"
#include "object_manipulator/place_execution/place_tester_fast.h"

#include "object_manipulator/tools/grasp_marker_publisher.h"
#include "object_manipulator/tools/exceptions.h"

using object_manipulation_msgs::GraspableObject;
using object_manipulation_msgs::PickupGoal;
using object_manipulation_msgs::PickupResult;
using object_manipulation_msgs::PickupFeedback;
using object_manipulation_msgs::PlaceGoal;
using object_manipulation_msgs::PlaceResult;
using object_manipulation_msgs::PlaceFeedback;
using object_manipulation_msgs::ManipulationResult;
using object_manipulation_msgs::GraspResult;
using object_manipulation_msgs::PlaceLocationResult;
using object_manipulation_msgs::getGraspResultInfo;
using object_manipulation_msgs::getPlaceLocationResultInfo;
using object_manipulation_msgs::Grasp;
using object_manipulation_msgs::GraspPlanningAction;

namespace object_manipulator {

ObjectManipulator::ObjectManipulator() :
  priv_nh_("~"),
  root_nh_(""),
  grasp_planning_actions_("", "", false, false),
  marker_pub_(NULL)
{
  bool publish_markers = true;
  if (publish_markers)
  {
    marker_pub_ = new GraspMarkerPublisher();
  }

  //old style executors
  grasp_executor_with_approach_ = new GraspExecutorWithApproach(marker_pub_);
  reactive_grasp_executor_ = new ReactiveGraspExecutor(marker_pub_);
  unsafe_grasp_executor_ = new UnsafeGraspExecutor(marker_pub_);
  place_executor_ = new PlaceExecutor(marker_pub_);
  reactive_place_executor_ = new ReactivePlaceExecutor(marker_pub_);

  //new syle executors
  
  grasp_tester_with_approach_ = new GraspTesterWithApproach;
  grasp_tester_with_approach_->setMarkerPublisher(marker_pub_);
  grasp_tester_fast_ = new GraspTesterFast;
  grasp_tester_fast_->setMarkerPublisher(marker_pub_);
  unsafe_grasp_tester_ = new UnsafeGraspTester;
  unsafe_grasp_tester_->setMarkerPublisher(marker_pub_);
  standard_grasp_performer_ = new StandardGraspPerformer;
  standard_grasp_performer_->setMarkerPublisher(marker_pub_);
  reactive_grasp_performer_ = new ReactiveGraspPerformer;
  reactive_grasp_performer_->setMarkerPublisher(marker_pub_);
  unsafe_grasp_performer_ = new UnsafeGraspPerformer;
  unsafe_grasp_performer_->setMarkerPublisher(marker_pub_);
 
  standard_place_tester_ = new PlaceTesterFast;
  standard_place_tester_->setMarkerPublisher(marker_pub_);
  standard_place_performer_ = new StandardPlacePerformer;
  standard_place_performer_->setMarkerPublisher(marker_pub_);
  reactive_place_performer_ = new ReactivePlacePerformer;
  reactive_place_performer_->setMarkerPublisher(marker_pub_);

  priv_nh_.param<std::string>("default_cluster_planner", default_cluster_planner_, "default_cluster_planner");
  priv_nh_.param<std::string>("default_database_planner", default_database_planner_, "default_database_planner");
  priv_nh_.param<std::string>("default_probabilistic_planner", default_probabilistic_planner_,
    "default_probabilistic_planner");
  priv_nh_.param<bool>("use_probabilistic_grasp_planner", use_probabilistic_planner_, false);
  priv_nh_.param<bool>("randomize_grasps", randomize_grasps_, false);

  ROS_INFO("Object manipulator ready. Default cluster planner: %s. Default database planner: %s.", 
	   default_cluster_planner_.c_str(), default_database_planner_.c_str());
  if(use_probabilistic_planner_)
  {
    ROS_INFO("Probabilistic planner enabled");
  }
  ROS_INFO_NAMED("manipulation","Object manipulator ready");
}

ObjectManipulator::~ObjectManipulator()
{
  delete marker_pub_;

  //old style executors
  delete grasp_executor_with_approach_;
  delete reactive_grasp_executor_;
  delete unsafe_grasp_executor_;
  delete place_executor_;
  delete reactive_place_executor_;

  //new style executors
  delete grasp_tester_fast_;
  delete grasp_tester_with_approach_;
  delete unsafe_grasp_tester_;
  delete standard_grasp_performer_;
  delete reactive_grasp_performer_;
  delete unsafe_grasp_performer_;

  delete standard_place_tester_;
  delete standard_place_performer_;
  delete reactive_place_performer_;
}

void ObjectManipulator::graspFeedback(
                                 actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> *action_server,
                                 size_t tested_grasps, size_t current_grasp)
{
  PickupFeedback feedback;
  feedback.total_grasps = grasp_container_.size();
  feedback.current_grasp = current_grasp + tested_grasps;
  action_server->publishFeedback(feedback);
}

void ObjectManipulator::graspPlanningFeedbackCallback(
                                             const object_manipulation_msgs::GraspPlanningFeedbackConstPtr &feedback)
{
  ROS_DEBUG_STREAM_NAMED("manipulation", "Feedback from planning action, total grasps: " << feedback->grasps.size());
  grasp_container_.addGrasps(feedback->grasps);
}

void ObjectManipulator::graspPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
                                             const object_manipulation_msgs::GraspPlanningResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG_STREAM_NAMED("manipulation", "Final result from planning action, total grasps: " << result->grasps.size());
    grasp_container_.addGrasps(result->grasps);
  }
  else
  {
    ROS_ERROR("Grasp planning action did not succeed");
  }
}

void ObjectManipulator::pickup(const PickupGoal::ConstPtr &pickup_goal,
			       actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> *action_server)
{
  //the result that will be returned
  PickupResult result;

  //we are making some assumptions here. We are assuming that the frame of the cluster is the
  //cannonical frame of the system, so here we check that the frames of all recognitions
  //agree with that. 
  //TODO find a general solution for this problem
  if ( pickup_goal->target.cluster.points.size() > 0)
  {
    for (size_t i=0; i<pickup_goal->target.potential_models.size(); i++)
    {
      if ( pickup_goal->target.potential_models[i].pose.header.frame_id != pickup_goal->target.cluster.header.frame_id)
      {
        ROS_ERROR("Target object recognition result(s) not in the same frame as the cluster");
        result.manipulation_result.value = ManipulationResult::ERROR;
        action_server->setAborted(result);
        return;
      }
    }
  }
  
  //populate the grasp container
  grasp_container_.clear();
  bool using_planner_action;
  std::string planner_action;
  if (!pickup_goal->desired_grasps.empty())
  {
    //use the requested grasps, if any
    grasp_container_.addGrasps(pickup_goal->desired_grasps);
    using_planner_action = false;
  }
  else
  {
    if (use_probabilistic_planner_)
    {
        ROS_INFO("Using probabilistic planner");
        // use the default probabilistic planner
        planner_action = default_probabilistic_planner_;
    }
    else
    {
        // decide which grasp planner to call depending on the type of object
        if (!pickup_goal->target.potential_models.empty())
            planner_action = default_database_planner_;
        else
            planner_action = default_cluster_planner_;        
    }

    //call the planner action, which will populate the grasp container as feedback arrives
    object_manipulation_msgs::GraspPlanningGoal goal;
    goal.arm_name = pickup_goal->arm_name;
    goal.target = pickup_goal->target;
    goal.collision_object_name = pickup_goal->collision_object_name;
    goal.collision_support_surface_name = pickup_goal->collision_support_surface_name;
    goal.movable_obstacles = pickup_goal->movable_obstacles;
    try
    {
      grasp_planning_actions_.client(planner_action).sendGoal(goal, 
                                          boost::bind(&ObjectManipulator::graspPlanningDoneCallback, this, _1, _2),
                                          actionlib::SimpleActionClient<GraspPlanningAction>::SimpleActiveCallback(), 
                                          boost::bind(&ObjectManipulator::graspPlanningFeedbackCallback, this, _1));
    }
    catch (ServiceNotFoundException &ex)
    {
      ROS_ERROR("Planning action %s not found", planner_action.c_str());
      result.manipulation_result.value = ManipulationResult::ERROR;
      action_server->setAborted(result);
      return;
    }
    using_planner_action = true;
  }
  ScopedGoalCancel<GraspPlanningAction> goal_cancel(NULL);
  if (using_planner_action)
  {
    goal_cancel.setClient(&grasp_planning_actions_.client(planner_action));
  }
  //decide which grasp tester and performer will be used
  GraspTester *grasp_tester;
  GraspPerformer *grasp_performer;
  if (pickup_goal->ignore_collisions) 
  {
    grasp_tester = unsafe_grasp_tester_;
    grasp_performer = unsafe_grasp_performer_;
  }
  else 
  {
    grasp_tester = grasp_tester_fast_; //grasp_tester_with_approach_;
    if (pickup_goal->use_reactive_execution)
    {
      grasp_performer = reactive_grasp_performer_;
    }
    else
    {
      grasp_performer = standard_grasp_performer_;
    }
  }

  grasp_tester->setInterruptFunction(
               boost::bind(&actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction>::isPreemptRequested,
                           action_server));
  grasp_performer->setInterruptFunction(
               boost::bind(&actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction>::isPreemptRequested,
                           action_server));

  //PROF_RESET_ALL;
  //PROF_START_TIMER(TOTAL_PICKUP_TIMER);

  ros::WallDuration dur(1.0);
  dur.sleep();

  arm_navigation_msgs::OrderedCollisionOperations emp_coll;
  std::vector<arm_navigation_msgs::LinkPadding> link_padding;
  mechInterface().getPlanningScene(emp_coll, link_padding);

  std::map<unsigned int, unsigned int> results;

  ros::WallTime start = ros::WallTime::now();

  //try the grasps in the list until one succeeds
  result.manipulation_result.value = ManipulationResult::UNFEASIBLE;
  try
  {
    size_t tested_grasps = 0;
    while (1)
    {
      if (action_server->isPreemptRequested()) throw InterruptRequestedException();

      ROS_DEBUG_STREAM_NAMED("manipulation", "Object manipulator: getting grasps beyond " << tested_grasps);
      std::vector<object_manipulation_msgs::Grasp> new_grasps = grasp_container_.getGrasps(tested_grasps);
      if ( new_grasps.empty() )
      { 
        if ( using_planner_action && (grasp_planning_actions_.client(planner_action).getState() == 
                                      actionlib::SimpleClientGoalState::ACTIVE || 
                                      grasp_planning_actions_.client(planner_action).getState() == 
                                      actionlib::SimpleClientGoalState::PENDING) )
        {
          ROS_DEBUG_NAMED("manipulation", "Object manipulator: waiting for planner action to provide grasps");
          ros::Duration(0.25).sleep();
          continue;
        }
        else
        {
          ROS_INFO("Object manipulator: all grasps have been tested");
          break;
        }
      }
      grasp_tester->setFeedbackFunction(boost::bind(&ObjectManipulator::graspFeedback, 
                                                    this, action_server, tested_grasps,  _1));
      //test a batch of grasps
      std::vector<GraspExecutionInfo> execution_info;
      grasp_tester->testGrasps(*pickup_goal, new_grasps, execution_info, !pickup_goal->only_perform_feasibility_test);
      if (execution_info.empty()) throw GraspException("grasp tester provided empty ExecutionInfo");
      if (execution_info.empty()) throw GraspException("grasp tester provided empty ExecutionInfo");
      //try to perform them
      if (!pickup_goal->only_perform_feasibility_test)
      {
        ROS_DEBUG_NAMED("manipulation", "Attempting to perform grasps");
        grasp_performer->performGrasps(*pickup_goal, new_grasps, execution_info);
      }
      if (execution_info.empty()) throw GraspException("grasp performer provided empty ExecutionInfo");
      //copy information about tested grasps over in result
      for (size_t i=0; i<execution_info.size(); i++)
      {
        result.attempted_grasps.push_back(new_grasps[i]);
        result.attempted_grasp_results.push_back(execution_info[i].result_);
      }
      //see if we're done
      if (execution_info.back().result_.result_code == GraspResult::SUCCESS)
      {
        result.manipulation_result.value = ManipulationResult::SUCCESS;
        if (!pickup_goal->only_perform_feasibility_test)
        {
          ROS_DEBUG_NAMED("manipulation", "Grasp reports success");
          result.grasp = new_grasps.at( execution_info.size() -1 );
          action_server->setSucceeded(result);
          return;
        }
      }
      //see if continuation is possible
      if (!execution_info.back().result_.continuation_possible)
      {
        if (pickup_goal->only_perform_feasibility_test)
        {
          ROS_ERROR("Continuation impossible when performing feasibility test");
        }
        result.grasp = new_grasps.at( execution_info.size() -1 );
        if (execution_info.back().result_.result_code == GraspResult::LIFT_FAILED)
          result.manipulation_result.value = ManipulationResult::LIFT_FAILED;
        else
          result.manipulation_result.value = ManipulationResult::FAILED;
        action_server->setAborted(result);
        return;
      }
      //remember how many grasps we've tried
      tested_grasps += execution_info.size();
    }
    //all the grasps have been tested
    if (pickup_goal->only_perform_feasibility_test && result.manipulation_result.value == ManipulationResult::SUCCESS)
    {
      action_server->setSucceeded(result);
    }
    else
    {
      action_server->setAborted(result);
    }
    return;
  }
  catch (InterruptRequestedException &ex)
  {
    ROS_DEBUG_NAMED("manipulation","Pickup goal preempted");
    action_server->setPreempted();
    return;
  }
  catch (MoveArmStuckException &ex)
  {
    ROS_ERROR("Grasp aborted because move_arm is stuck");
    result.manipulation_result.value = ManipulationResult::ARM_MOVEMENT_PREVENTED;
    action_server->setAborted(result);
    return;
  }
  catch (GraspException &ex)
  {
    ROS_ERROR("Grasp error; exception: %s", ex.what());
    result.manipulation_result.value = ManipulationResult::ERROR;
    action_server->setAborted(result);
    return;
  }
}

void ObjectManipulator::placeFeedback(
                                 actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> *action_server,
                                 size_t tested_places, size_t total_places, size_t current_place)
{
  PlaceFeedback feedback;
  feedback.total_locations = total_places;
  feedback.current_location = current_place + tested_places;
  action_server->publishFeedback(feedback);
}

void ObjectManipulator::place(const object_manipulation_msgs::PlaceGoal::ConstPtr &place_goal,
			      actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> *action_server)
{
  PlaceResult result;
  PlaceTester *place_tester = standard_place_tester_;
  PlacePerformer *place_performer = standard_place_performer_;
  if (place_goal->use_reactive_place) place_performer = reactive_place_performer_;
  
  place_tester->setInterruptFunction(
               boost::bind(&actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction>::isPreemptRequested,
                           action_server));
  place_performer->setInterruptFunction(
               boost::bind(&actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction>::isPreemptRequested,
                           action_server));

  std::vector<geometry_msgs::PoseStamped> place_locations = place_goal->place_locations;
  try
  {
    std::vector<PlaceExecutionInfo> execution_info;
    result.manipulation_result.value = ManipulationResult::UNFEASIBLE;
    size_t tested_places = 0;
    while (!place_locations.empty())
    {
      if (action_server->isPreemptRequested()) throw InterruptRequestedException();
      place_tester->setFeedbackFunction(boost::bind(&ObjectManipulator::placeFeedback, 
                                                    this, action_server, tested_places, place_locations.size(), _1));
      //test a batch of locations
      place_tester->testPlaces(*place_goal, place_locations, execution_info, 
                               !place_goal->only_perform_feasibility_test);
      if (execution_info.empty()) throw GraspException("place tester provided empty ExecutionInfo");
      //try to perform them
      if (!place_goal->only_perform_feasibility_test)
      {
        place_performer->performPlaces(*place_goal, place_locations, execution_info);
      }
      if (execution_info.empty()) throw GraspException("place performer provided empty ExecutionInfo");
      //copy information about tested places over in result
      std::vector<geometry_msgs::PoseStamped>::iterator it = place_locations.begin();
      for (size_t i=0; i<execution_info.size(); i++)
      {
        result.attempted_locations.push_back(place_locations[i]);
        result.attempted_location_results.push_back(execution_info[i].result_);
        it++;
      }
      //see if we're done
      if (execution_info.back().result_.result_code == PlaceLocationResult::SUCCESS)
      {
        result.manipulation_result.value = ManipulationResult::SUCCESS;
        if (!place_goal->only_perform_feasibility_test)
        {
          result.place_location = place_locations.at( execution_info.size() - 1 );
          action_server->setSucceeded(result);
          return;
        }
      }
      //see if continuation is possible
      if (!execution_info.back().result_.continuation_possible)
      {
        if (place_goal->only_perform_feasibility_test)
        {
          ROS_ERROR("Continuation impossible when performing feasibility test");
        }
        result.place_location = place_locations.at( execution_info.size() - 1 );
        if (execution_info.back().result_.result_code == PlaceLocationResult::RETREAT_FAILED)
          result.manipulation_result.value = ManipulationResult::RETREAT_FAILED;
        else
          result.manipulation_result.value = ManipulationResult::FAILED;
        action_server->setAborted(result);
        return;
      }
      //clear all the grasps we've tried
      tested_places += execution_info.size();
      execution_info.clear();      
      place_locations.erase(place_locations.begin(), it);
    }
    //all the grasps have been tested
    if (place_goal->only_perform_feasibility_test && result.manipulation_result.value == ManipulationResult::SUCCESS)
    {
      action_server->setSucceeded(result);
    }
    else
    {
      action_server->setAborted(result);
    }
    return;
  }
  catch (InterruptRequestedException &ex)
  {
    ROS_DEBUG_NAMED("manipulation","Place goal preempted");
    action_server->setPreempted();
    return;
  }
  catch (MoveArmStuckException &ex)
  {
    ROS_ERROR("Place aborted because move_arm is stuck");
    result.manipulation_result.value = ManipulationResult::ARM_MOVEMENT_PREVENTED;
    action_server->setAborted(result);
    return;
  }
  catch (GraspException &ex)
  {
    ROS_ERROR("Place error; exception: %s", ex.what());
    result.manipulation_result.value = ManipulationResult::ERROR;
    action_server->setAborted(result);
    return;
  } 
}

}

