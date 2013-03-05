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
#ifndef _CAMERA_CONFIGURATIONS_H_
#define _CAMERA_CONFIGURATIONS_H_

#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

#include "object_manipulator/tools/exceptions.h"

#include "configuration_loader.h"

namespace object_manipulator {

class CameraConfigurations : public ConfigurationLoader
{
private:

public:
  CameraConfigurations() {}

  inline std::vector< double > get_camera_pose( const std::string &position_name )
  {
    ROS_WARN("This functionality is deprecated. Use get_camera_placement instead.");
    std::string name = "/im_camera_configurations/" + position_name;
    std::vector<double> values = getVectorDoubleParam(name);
    if (values.size() != 6) throw BadParamException(name);
    return values;
  }

  inline bool get_camera_placement( const std::string &position_name,
                                    geometry_msgs::PointStamped &eye,
                                    geometry_msgs::PointStamped &focus,
                                    geometry_msgs::Vector3Stamped &up)
  {
    try
    {
      std::string name = "/im_camera_placements/" + position_name;
      //std::string frame_id = "";
      eye.header.stamp = ros::Time(0);
      eye.header.frame_id = getStringParam(name + "/eye/frame_id");
      focus.header.stamp = ros::Time(0);
      focus.header.frame_id = getStringParam(name + "/focus/frame_id");
      up.header.stamp = ros::Time(0);
      up.header.frame_id = getStringParam(name + "/up/frame_id");

      std::vector<double> eye_values = getVectorDoubleParam(name + "/eye/point");
      if (eye_values.size() != 3) throw BadParamException(name + "/eye/point");
      eye.point.x = eye_values[0];
      eye.point.y = eye_values[1];
      eye.point.z = eye_values[2];

      std::vector<double> focus_values = getVectorDoubleParam(name + "/focus/point");
      if (focus_values.size() != 3) throw BadParamException(name + "/focus/point");
      focus.point.x = focus_values[0];
      focus.point.y = focus_values[1];
      focus.point.z = focus_values[2];

      std::vector<double> up_values = getVectorDoubleParam(name + "/up/vector");
      if (up_values.size() != 3) throw BadParamException(name + "/up/vector");
      up.vector.x = up_values[0];
      up.vector.y = up_values[1];
      up.vector.z = up_values[2];

      return true;
    }
    catch(object_manipulator::BadParamException &ex)
    {
      ROS_ERROR("%s \n (Parameters that represent doubles or lists of doubles need to have decimal points.)", ex.what());
    }
    catch(object_manipulator::MissingParamException &ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    return false;
  }

}; // end of class definition

//! Returns a CameraConfigurations singleton
inline CameraConfigurations& cameraConfigurations()
{
  static CameraConfigurations camera_configs;
  return camera_configs;
}

} //namespace object_manipulator

#endif
