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
#ifndef _ARM_CONFIGURATIONS_H_
#define _ARM_CONFIGURATIONS_H_

#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/Vector3.h>

#include "object_manipulator/tools/exceptions.h"

#include "configuration_loader.h"

namespace object_manipulator {

class ArmConfigurations : public ConfigurationLoader
{
 private:

 public:
 ArmConfigurations() {}

  inline std::vector< double > position(std::string arm_name, std::string position)
  {
    std::string name = "/arm_configurations/" + position + "/position/" + arm_name;
    std::vector<double> values = getVectorDoubleParam(name);
    if ( values.size() != 7 )  throw BadParamException(name);
    return values;
  }

  inline std::vector< std::vector<double> > trajectory(std::string arm_name, std::string position)
  {
    std::string name = "/arm_configurations/" + position + "/trajectory/" + arm_name;
    std::vector<double> values = getVectorDoubleParam(name);
    if ( values.size() % 7 != 0 )  throw BadParamException(name);
    std::vector< std::vector<double> > traj;
    int waypoints = values.size() / 7;
    traj.resize( waypoints );
    for(int i = 0; i < waypoints; i++)
    {
      std::vector<double> pos;
      traj[i].insert(pos.begin(), values.begin() + i*7, values.begin() + (i+1)*7 );
    }
    return traj;
  }

};

//! Returns a hand description singleton
inline ArmConfigurations& armConfigurations()
{
  static ArmConfigurations arm_configs;
  return arm_configs;
}

} //namespace object_manipulator

#endif
