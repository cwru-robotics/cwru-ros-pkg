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
#ifndef _CONFIGURATION_LOADER_H_
#define _CONFIGURATION_LOADER_H_

#include <ros/ros.h>

#include "object_manipulator/tools/exceptions.h"

namespace object_manipulator {

class ConfigurationLoader
{
  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

public:
  //! Fetch a single string param
  inline std::string getStringParam(std::string name)
  {
    std::string value;
    if (!root_nh_.getParamCached(name, value)) throw MissingParamException(name);
    //ROS_INFO_STREAM("Hand description param " << name << " resolved to " << value);
    return value;
  }

  //! Fetch a vector of strings param
  inline std::vector<std::string> getVectorParam(std::string name)
  {
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list)) throw MissingParamException(name);
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) throw BadParamException(name);
    //ROS_INFO_STREAM("Hand description vector param " << name << " resolved to:");
    std::vector<std::string> values;
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeString) throw BadParamException(name);
      values.push_back( static_cast<std::string>(list[i]) );
      //ROS_INFO_STREAM("  " << values.back());
    }
    return values;	
  }

  //! Fetch a vector of doubles param
  inline std::vector<double> getVectorDoubleParam(std::string name)
  {
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list)) throw MissingParamException(name);
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) throw BadParamException(name);
    std::vector<double> values;
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) throw BadParamException(name);
      values.push_back( static_cast<double>(list[i]) );
    }
    return values;
  }

  //! Fetch a double param
  inline double getDoubleParam(std::string name)
  {
    double value;
    if (!root_nh_.getParamCached(name, value)) throw MissingParamException(name);
    return value;
  }

  //! Fetch a bool param
  inline bool getBoolParam(std::string name)
  {
    bool value;
    if (!root_nh_.getParamCached(name, value)) throw MissingParamException(name);
    return value;
  }

  //! We should maybe create a instance() member function and make the contructor protected...
  ConfigurationLoader() : root_nh_("~") {}

};

//! Returns a configuration loader singleton
inline ConfigurationLoader& configurationLoader()
{
  static ConfigurationLoader singleton;
  return singleton;
}

} //namespace object_manipulator

#endif
