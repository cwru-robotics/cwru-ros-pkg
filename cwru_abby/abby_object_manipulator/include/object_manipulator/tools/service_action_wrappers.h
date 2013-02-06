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
#ifndef _SERVICE_ACTION_WRAPPERS_H_
#define _SERVICE_ACTION_WRAPPERS_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <map>

#include "object_manipulator/tools/exceptions.h"

namespace object_manipulator {

//! Wrapper class for service clients to perform initialization on first use
/*! When the client is first used, it will check for the existence of the service
  and wait until the service becomes available.
 */
template <class ServiceDataType>
class ServiceWrapper
{
 private:
  //! Has the service client been initialized or not
  bool initialized_;
  //! The name of the service
  std::string service_name_;
  //! The node handle to be used when initializing services
  ros::NodeHandle nh_;
  //! The actual client handle
  ros::ServiceClient client_;
  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
 public:
 ServiceWrapper(std::string service_name) : initialized_(false), 
    service_name_(service_name),
    nh_("")
    {}
  
  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Returns reference to client. On first use, initializes (and waits for) client. 
  ros::ServiceClient& client(ros::Duration timeout = ros::Duration(5.0)) 
  {
    if (!initialized_)
    {
      ros::Duration ping_time = ros::Duration(1.0);
      if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
      ros::Time start_time = ros::Time::now();
      while (1)
      {
	if (ros::service::waitForService(service_name_, ping_time)) break;
	ROS_INFO_STREAM("Waiting for service " << service_name_);
        if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
	if (!ros::ok()) throw ServiceNotFoundException(service_name_);
	ros::Time current_time = ros::Time::now();
	if (timeout > ros::Duration(0) && current_time - start_time >= timeout) 
	  throw ServiceNotFoundException(service_name_);
      }
      client_ = nh_.serviceClient<ServiceDataType>(service_name_);	
      initialized_ = true;
    }
    return client_;
  }

  bool isInitialized() const {return initialized_;}
};

//! A wrapper for multiple instances of a given service, one for each arm in the system
/*! This function performs mapping from an arm_name to a service for that arm. 

  It obtains the name of the service by adding a prefix and/or suffix to the name of the arm.

  For each arm, it also performs "wait on first use service". When the service for a given arm
  name is first requested, it will wait for the service, then create the client and return it. 
  It will also remember the client, so that on subsequent calls the client is returned directly
  without additional waiting.
 */
template <class ServiceDataType>
class MultiArmServiceWrapper
{
 private:
  //! The node handle used to create services
  ros::NodeHandle nh_;
  //! Prefix attached to arm name to get service name
  std::string prefix_;
  //! Suffix attached to arm name to get service name
  std::string suffix_;
 
  typedef std::map<std::string, ros::ServiceClient> map_type; 

  //! The list of clients already created, mapped to service names
  map_type clients_;
  //! Whether the resulting name should first be resolved using the node handle
  /*! Use this if you are remapping service names. */
  bool resolve_names_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;

 public:
  //! Sets the node handle, prefix and suffix
 MultiArmServiceWrapper(std::string prefix, std::string suffix, bool resolve_names) : 
  nh_(""), prefix_(prefix), suffix_(suffix), resolve_names_(resolve_names)
  {}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  //! Returns a service client for the requested arm
  /*! Service name is obtained as prefix + arm_name + suffix.
    On first request for a given arm, a service client will be initialized, and the service will
    be waited for. On subsequent calls for the same arm, the service is returned directly.
  */
  ros::ServiceClient& client(std::string arm_name, ros::Duration timeout = ros::Duration(5.0))
    {
      //compute the name of the service
      std::string client_name = prefix_ + arm_name + suffix_;

      //check if the service is already there
      map_type::iterator it = clients_.find(client_name);
      if ( it != clients_.end() ) 
      {
	return it->second;
      }

      std::string service_name = client_name;
      if (resolve_names_) service_name = nh_.resolveName(client_name);

      //new service; wait for it
      ros::Duration ping_time = ros::Duration(1.0);
      if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
      ros::Time start_time = ros::Time::now();
      while(1)
      {
	if (ros::service::waitForService(service_name, ping_time)) break;
        if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
	if (!ros::ok()) throw ServiceNotFoundException(client_name + " remapped to " + service_name);
	ros::Time current_time = ros::Time::now();
	if (timeout > ros::Duration(0) && current_time - start_time >= timeout) 
	  throw ServiceNotFoundException(client_name + " remapped to " + service_name);
	ROS_INFO_STREAM("Waiting for service " << client_name << " remapped to " << service_name);
      }

      //insert new service in list
      std::pair<map_type::iterator, bool> new_pair;
      new_pair = clients_.insert(std::pair<std::string, ros::ServiceClient>
				 (client_name, nh_.serviceClient<ServiceDataType>(service_name) ) );

      //and return it
      return new_pair.first->second;
    }
};

//! Cancels all goals of the client when it goes out of scope
template <class ActionDataType>
class ScopedGoalCancel
{
private:
  actionlib::SimpleActionClient<ActionDataType> *client_;
public:
  ScopedGoalCancel(actionlib::SimpleActionClient<ActionDataType> *client) : client_(client) {}
  void setClient(actionlib::SimpleActionClient<ActionDataType> *client){client_ = client;}
  ~ScopedGoalCancel(){if (client_) client_->cancelAllGoals();}
};

//! Wrapper class for SimpleActionClient that checks server availability on first use
template <class ActionDataType>
class ActionWrapper
{
 private:
  //! Has the service client been initialized or not
  bool initialized_;
  //! The name of the action
  std::string action_name_;
  //! The remapped name/topic of the action
  std::string remapped_name_;
  //! The node handle to be used when initializing services
  ros::NodeHandle nh_;  
  //! The actual action client
  actionlib::SimpleActionClient<ActionDataType> client_;
  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;
 public:
 ActionWrapper(std::string action_name, bool spin_thread) : initialized_(false),
    action_name_(action_name),
    remapped_name_("*name failed to remap!*"),
    nh_(""),
    client_(nh_, action_name, spin_thread) {}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  actionlib::SimpleActionClient<ActionDataType>& client(ros::Duration timeout = ros::Duration(5.0))
  {
    if (!initialized_)
    {
      // aleeper: Added this to aid with remapping debugging.
      remapped_name_ = nh_.resolveName(action_name_, true);
      ros::Duration ping_time = ros::Duration(1.0);
      if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
      ros::Time start_time = ros::Time::now();
      while (1)
      {
	if (client_.waitForServer(ping_time)) break;
        if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
	if (!ros::ok()) throw ServiceNotFoundException(action_name_);
	ros::Time current_time = ros::Time::now();
	if (timeout > ros::Duration(0) && current_time - start_time >= timeout) 
	  throw ServiceNotFoundException(action_name_);
	ROS_INFO_STREAM("Waiting for action client: " << action_name_ << " remapped to " << remapped_name_);
      }
      initialized_ = true;
    }
    return client_;
  }

  bool waitForResult(const ros::Duration &timeout=ros::Duration(0,0))
  {
    ros::Duration ping_time = ros::Duration(5.0);
    if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
    ros::Time start_time = ros::Time::now();
    while (1)
    {
      if (client().waitForResult(ping_time)) return true;
      if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
      //we should probably throw something else here
      if (!ros::ok()) throw ServiceNotFoundException(action_name_);
      ros::Time current_time = ros::Time::now();
      if (timeout > ros::Duration(0) && current_time - start_time >= timeout) return false;
      if (!client().isServerConnected()) return false;
      ROS_INFO_STREAM("Waiting for result from action client: " << action_name_ << " remapped to " << remapped_name_);
    }
  }

  bool isInitialized() const {return initialized_;}
};


//! A wrapper for multiple instances of a topic publisher, one for each arm in the system
/*! This function performs mapping from an arm_name to a topic for that arm.
  It obtains the name of the action by adding a prefix and/or suffix to the name of the arm.
*/
template <class TopicDataType>
class MultiArmTopicWrapper
{ 
 private:
  //! The node handle used to create the publisher
  ros::NodeHandle nh_;

  //! Prefix attached to arm name to get topic name
  std::string prefix_;

  //! Suffix attached to arm name to get topic name
  std::string suffix_;

  typedef std::map<std::string, ros::Publisher> map_type; 

  //! The list of publishers already created, mapped to service names
  map_type publishers_;
  
  //! Whether the resulting name should first be resolved using the node handle
  /*! Use this if you are remapping topic names. */
  bool resolve_names_;

 public:
  //! Sets the node handle, prefix and suffix
 MultiArmTopicWrapper(std::string prefix, std::string suffix, bool resolve_names) : 
  nh_(""), prefix_(prefix), suffix_(suffix), resolve_names_(resolve_names)
  {}

  //! Returns a publisher for the requested arm
  /*! Topic name is obtained as prefix + arm_name + suffix.
    On first request for a given arm, a publisher will be initialized.
    On subsequent calls for the same arm, the publisher is returned directly.
  */
  ros::Publisher& publisher(std::string arm_name, ros::Duration timeout = ros::Duration(5.0))
  {
    //compute the name of the topic
    std::string topic_name = prefix_ + arm_name + suffix_;
    
    //check if the publisher is already there
    map_type::iterator it = publishers_.find(topic_name);
    if ( it != publishers_.end() ) return it->second;
    
    //resolve the name if needed
    std::string resolved_topic_name = topic_name;
    if (resolve_names_) resolved_topic_name = nh_.resolveName(topic_name);
    
    //insert new publisher in list
    std::pair<map_type::iterator, bool> new_pair;
    new_pair = publishers_.insert(std::pair<std::string, ros::Publisher>
                                  (topic_name, nh_.advertise<TopicDataType>(resolved_topic_name, 100) ) );
    
    //and return it
    return new_pair.first->second;
  }
};


//! A wrapper for multiple instances of a given action, one for each arm in the system
/*! This function performs mapping from an arm_name to a action for that arm. 

  It obtains the name of the action by adding a prefix and/or suffix to the name of the arm.

  For each arm, it also performs "wait on first use". When the action for a given arm
  name is first requested, it will wait for the action, then create the client and return it. 
  It will also remember the client, so that on subsequent calls the client is returned directly
  without additional waiting.
 */
template <class ActionDataType>
class MultiArmActionWrapper
{
 private:
  //! The node handle used to create actions
  ros::NodeHandle nh_;

  //! Prefix attached to arm name to get action name
  std::string prefix_;

  //! Suffix attached to arm name to get action name
  std::string suffix_;
  
  //! Parameter to be passed on to the action client
  bool spin_thread_;

  /*! It seems the SimpleActionClient is non-copyable, so I could not store them directly in the map. */
  typedef std::map<std::string, actionlib::SimpleActionClient<ActionDataType>* > map_type;

  //! The list of clients already created, mapped to action names
  map_type clients_;

  //! Whether the resulting name should first be resolved using the node handle
  /*! Use this if you are remapping service names. */
  bool resolve_names_;

  //! Function used to check for interrupts
  boost::function<bool()> interrupt_function_;

 public:
  //! Sets the node handle, prefix and suffix
 MultiArmActionWrapper(std::string prefix, std::string suffix, bool spin_thread, bool resolve_names) : 
  nh_(""), prefix_(prefix), suffix_(suffix), spin_thread_(spin_thread), resolve_names_(resolve_names)
  {}

  //! Sets the interrupt function
  void setInterruptFunction(boost::function<bool()> f){interrupt_function_ = f;}

  ~MultiArmActionWrapper()
  {
    for (typename map_type::iterator it = clients_.begin(); it!=clients_.end(); it++)
    {
      delete it->second;
    }
  }

  //! Returns a action client for the requested arm
  /*! Action name is obtained as prefix + arm_name + suffix.
    On first request for a given arm, a action client will be initialized, and the action will
    be waited for. On subsequent calls for the same arm, the action is returned directly.
  */
  actionlib::SimpleActionClient<ActionDataType>& client(std::string arm_name, 
							ros::Duration timeout = ros::Duration(5.0))
    {
      //compute the name of the action
      std::string client_name = prefix_ + arm_name + suffix_;

      //check if the action client is already there      
      typename map_type::iterator it = clients_.find(client_name);
      if ( it != clients_.end() ) 
      {
	return *(it->second);
      }
      
      std::string action_name = client_name;
      if (resolve_names_) action_name = nh_.resolveName(client_name);

      //create new action client and insert in map
      actionlib::SimpleActionClient<ActionDataType>* new_client = 
	new actionlib::SimpleActionClient<ActionDataType>(nh_, action_name, spin_thread_ );

      //wait for the server
      ros::Duration ping_time = ros::Duration(1.0);
      if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
      ros::Time start_time = ros::Time::now();
      while (1)
      {
	if (new_client->waitForServer(ping_time)) break;
        if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
	if (!ros::ok()) throw ServiceNotFoundException(client_name + " remapped to " + action_name);
	ros::Time current_time = ros::Time::now();
	if (timeout > ros::Duration(0) && current_time - start_time >= timeout) 
	  throw ServiceNotFoundException(client_name + " remapped to " + action_name);
	ROS_INFO_STREAM("Waiting for action client " << client_name << ", remapped to " << action_name);
      }

      //insert new client in map
      std::pair< typename map_type::iterator, bool> new_pair = 
	clients_.insert( std::pair<std::string,actionlib::SimpleActionClient<ActionDataType>* >
			 (client_name, new_client ) );

      //and return it
      return *(new_pair.first->second);      
    }

  //! The action client for the requested arm waits for result
  bool waitForResult(std::string arm_name, const ros::Duration &timeout=ros::Duration(0,0))
  {
    ros::Duration ping_time = ros::Duration(1.0);
    if (timeout > ros::Duration(0) && ping_time > timeout) ping_time = timeout;
    ros::Time start_time = ros::Time::now();
    while (1)
    {
      if (client(arm_name).waitForResult(ping_time)) return true;
      if (interrupt_function_ && interrupt_function_()) throw InterruptRequestedException();
      //we should probably throw something else here
      if (!ros::ok()) throw ServiceNotFoundException(arm_name);
      ros::Time current_time = ros::Time::now();
      if (timeout > ros::Duration(0) && current_time - start_time >= timeout) return false;
      if (!client(arm_name).isServerConnected()) return false;
      ROS_INFO_STREAM("Waiting for result from multi-arm action client on arm " << arm_name);
    }
  }

};


} //namespace object_manipulator

#endif
