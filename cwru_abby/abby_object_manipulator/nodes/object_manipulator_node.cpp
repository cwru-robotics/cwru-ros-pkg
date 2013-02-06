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

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

namespace object_manipulator {

static const std::string PICKUP_ACTION_NAME = "object_manipulator_pickup";
static const std::string PLACE_ACTION_NAME = "object_manipulator_place";

//! Wraps the Object Manipulator in a ROS API
class ObjectManipulatorNode
{
private:
  //! The private ROS node handle
  ros::NodeHandle priv_nh_;  

  //! The instance of the manipulator used to perform all tasks
  ObjectManipulator object_manipulator_;

  //! The action server for grasping
  actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> pickup_action_server_;

  //! The action server for placing
  actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> place_action_server_;

  //! Callback for the pickup action
  void pickupCallback(const object_manipulation_msgs::PickupGoal::ConstPtr &goal)
  {
    object_manipulator_.pickup(goal, &pickup_action_server_);
  }

  //! Callback for the placing action
  void placeCallback(const object_manipulation_msgs::PlaceGoal::ConstPtr &goal)
  {
    object_manipulator_.place(goal, &place_action_server_);
  }

public:
  ObjectManipulatorNode() : priv_nh_("~"),
			    pickup_action_server_( priv_nh_, PICKUP_ACTION_NAME, 
						   boost::bind(&ObjectManipulatorNode::pickupCallback, this, _1),
                                                   false),
			    place_action_server_( priv_nh_, PLACE_ACTION_NAME, 
						  boost::bind(&ObjectManipulatorNode::placeCallback, this, _1),
                                                  false)
  {
    pickup_action_server_.start();
    place_action_server_.start();
  }
};

} //namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_manipulator");
  object_manipulator::ObjectManipulatorNode node;
  ros::spin();
  return 0;
}
