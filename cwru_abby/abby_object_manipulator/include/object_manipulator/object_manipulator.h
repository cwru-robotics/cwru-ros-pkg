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

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <actionlib/server/simple_action_server.h>

#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <object_manipulation_msgs/GraspPlanningAction.h>

#include "object_manipulator/tools/service_action_wrappers.h"
#include "object_manipulator/tools/mechanism_interface.h"

namespace object_manipulator{

class PlaceExecutor;
class ReactivePlaceExecutor;
class ReactiveGraspExecutor;
class GraspExecutorWithApproach;
class UnsafeGraspExecutor;
class GraspMarkerPublisher;

class GraspTesterFast;

class GraspTester;
class GraspPerformer;
class PlaceTester;
class PlacePerformer;

class GraspContainer
{
private:
  std::vector<object_manipulation_msgs::Grasp> grasps_;
  boost::mutex mutex_;

public:
  size_t size() 
  {
    boost::mutex::scoped_lock lock(mutex_);
    return grasps_.size();
  }

  std::vector<object_manipulation_msgs::Grasp> getGrasps(size_t start) 
  {
    boost::mutex::scoped_lock lock(mutex_);
    std::vector<object_manipulation_msgs::Grasp> ret_grasps;
    if (start >= grasps_.size()) return ret_grasps;
    std::vector<object_manipulation_msgs::Grasp>::iterator it = grasps_.begin();
    for(size_t i=0; i<start; i++) it++;
    ret_grasps.insert(ret_grasps.begin(), it, grasps_.end());
    return ret_grasps;
  }

  void addGrasps(const std::vector<object_manipulation_msgs::Grasp> &new_grasps)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (new_grasps.size() <= grasps_.size())
    {
      ROS_WARN("No new grasps to add to container");
    }
    std::vector<object_manipulation_msgs::Grasp>::const_iterator it = new_grasps.begin();
    for (size_t i=0; i<grasps_.size(); i++) it++;
    grasps_.insert(grasps_.end(), it, new_grasps.end());
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(mutex_);
    grasps_.clear();
  }

  void popFrontGrasps(size_t n)
  {
    if (n==0) return;
    boost::mutex::scoped_lock lock(mutex_);
    if (n>grasps_.size())
    {
      ROS_WARN("Grasp container requested to pop more grasps than it has");
      grasps_.clear();
      return;
    }
    std::vector<object_manipulation_msgs::Grasp>::iterator it = grasps_.begin();
    for(size_t i=0; i<n; i++) it++;
    grasps_.erase(grasps_.begin(), it);
  }
};

//! Oversees the grasping app; bundles together functionality in higher level calls
/*! Also wraps the functionality in action replies, with the actual server passed in 
  from the caller so we keep ROS instantiations somewhat separated.
*/
class ObjectManipulator
{
private:
  //! The private ROS node handle
  ros::NodeHandle priv_nh_;

  //! The root ROS node handle
  ros::NodeHandle root_nh_;

  //! Wrapper for multiple grasp planning services with different names
  /*! Not really a wrapper for multiple arms, but a wrapper for multiple services
    with different names but fulfilling the same task.*/
  MultiArmActionWrapper<object_manipulation_msgs::GraspPlanningAction> grasp_planning_actions_;

  //! Publisher for grasp markers, or NULL if publishing is disabled
  GraspMarkerPublisher *marker_pub_;

  //! Whether the order in which grasps are tried should be randomized first. Debug purposes.
  bool randomize_grasps_;

  GraspTester* grasp_tester_with_approach_;
  GraspTester* unsafe_grasp_tester_;
  GraspTesterFast* grasp_tester_fast_;
  GraspPerformer* standard_grasp_performer_;
  GraspPerformer* reactive_grasp_performer_;
  GraspPerformer* unsafe_grasp_performer_;
  PlaceTester* standard_place_tester_;
  PlacePerformer* standard_place_performer_;
  PlacePerformer* reactive_place_performer_;

  //! Instance of the grasp executor with approach
  GraspExecutorWithApproach* grasp_executor_with_approach_;

  //! Instance of the reactive grasp executor
  ReactiveGraspExecutor* reactive_grasp_executor_;

  //! Instance of the executor used for grasping without considering collisions
  UnsafeGraspExecutor* unsafe_grasp_executor_;

  //! Instance of the executor used for placing objects
  PlaceExecutor* place_executor_;

  //! Instance of the executor used for reactive placing of objects
  ReactivePlaceExecutor* reactive_place_executor_;

  //! The name of the service to be used by default for grasp planning on database objects
  std::string default_database_planner_;

  //! The name of the service to be used by default for grasp planning on point cloud objects
  std::string default_cluster_planner_;

  //! The name of the service to be used by default for probabilistic grasp planning
  std::string default_probabilistic_planner_;
  bool use_probabilistic_planner_;

  //! A thread safe place to hold grasps returned by the planning action as feedback
  GraspContainer grasp_container_;

public:
  //! Initializes ros clients as needed
  ObjectManipulator();

  ~ObjectManipulator();

  //! Attempts to grasp the specified object
  void pickup(const object_manipulation_msgs::PickupGoal::ConstPtr &pickup_goal,
	      actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> *action_server);

  //! Attempts to place the specified object
  void place(const object_manipulation_msgs::PlaceGoal::ConstPtr &place_goal,
	     actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> *action_server);

  //! Provides feedback on the currently tested list of grasps
  void graspFeedback(actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> *action_server,
                     size_t tested_grasps, size_t current_grasp);

  //! Provides feedback on the currently tested list of places
  void placeFeedback(actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> *action_server,
                     size_t tested_places, size_t total_places, size_t current_place);

  //! Saves the grasps provided as feedback by planning action
  void graspPlanningFeedbackCallback(const object_manipulation_msgs::GraspPlanningFeedbackConstPtr &feedback);

  //! Saves the grasps provided as result by planning action
  void graspPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
                                 const object_manipulation_msgs::GraspPlanningResultConstPtr &result);

};

} //namespace grasping_app_executive
