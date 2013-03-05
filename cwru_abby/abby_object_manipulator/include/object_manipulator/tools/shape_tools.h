/*********************************************************************
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef _SHAPE_TOOLS_H_
#define _SHAPE_TOOLS_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include "object_manipulator/tools/msg_helpers.h"


namespace object_manipulator
{

namespace shapes
{

  /*!
  * \brief A representation of a box, using a frame and dimensions.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  class Box
  {
    public:
      Box(){   }

      Box(const tf::Pose &frame_in, const tf::Vector3 &dims_in) : frame(frame_in), dims(dims_in) {}

      Box(const tf::Vector3 &dims_in) : frame(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0))), dims(dims_in) {}

      inline void transform(const tf::Transform &T)   { frame = T * frame;  }

      std_msgs::Header header;
      tf::Pose frame;
      tf::Vector3 dims;
  };

  /*!
  * \brief A representation of a cylinder, using a frame and dimensions.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  class Cylinder
  {
    public:
      Cylinder(){   }

      Cylinder(const tf::Pose &frame_in, const tf::Vector3 &dims_in) : frame(frame_in), dims(dims_in) {}

      Cylinder(const tf::Vector3 &dims_in) : frame(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0))), dims(dims_in) {}

      inline void transform(const tf::Transform &T)   { frame = T * frame;  }

      std_msgs::Header header;
      tf::Pose frame;
      tf::Vector3 dims;
  };

  /*!
  * \brief A representation of a sphere/ellipsoid, using a frame and dimensions.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  class Sphere
  {
  public:
      Sphere(){   }

      Sphere(const tf::Pose &frame_in, const tf::Vector3 &dims_in) : frame(frame_in), dims(dims_in) {}

      Sphere(const tf::Vector3 &dims_in) : frame(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0))), dims(dims_in) {}

      inline void transform(const tf::Transform &T)   { frame = T * frame;  }

      std_msgs::Header header;
      tf::Pose frame;
      tf::Vector3 dims;
  };

  /*!
  * \brief A representation of an arrow, using a frame and dimensions.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  class Arrow
  {
  public:
      Arrow(){   }

      Arrow(const tf::Pose &frame_in, const tf::Vector3 &dims_in) : frame(frame_in), dims(dims_in) {}

      Arrow(const tf::Vector3 &dims_in) : frame(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0))), dims(dims_in) {}

      inline void transform(const tf::Transform &T)   { frame = T * frame;  }

      std_msgs::Header header;
      tf::Pose frame;
      tf::Vector3 dims;
  };

  /*!
  * \brief A container for a mesh, using a frame and dimensions.
  *
  * This description is displayed lower in the doxygen as an extended description along with
  * the above brief description.
  */
  class Mesh
  {
  public:
      Mesh(){   }

      Mesh(const char * mesh_source, const tf::Pose &frame_in, const tf::Vector3 &dims_in) : mesh_resource(mesh_source), frame(frame_in), dims(dims_in) {}

      Mesh(const char * mesh_source, const tf::Vector3 &dims_in) : mesh_resource(mesh_source), frame(tf::Pose(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0))), dims(dims_in) {}

      inline void transform(const tf::Transform &T)   { frame = T * frame;  }

      std_msgs::Header header;
      std::string mesh_resource;
      bool use_embedded_materials;
      tf::Pose frame;
      tf::Vector3 dims;

  };

} // namespace shapes


void drawCylinder(ros::Publisher &pub, const shapes::Cylinder &shape, const char *ns = "cylinder",
             int id = 0, const ros::Duration lifetime = ros::Duration(),
             const std_msgs::ColorRGBA color = msg::createColorMsg(0.5, 0.5, 0.5, 0.5),
             bool destroy = false, bool frame_locked = false);

void drawBox(ros::Publisher &pub, const shapes::Box &shape, const char *ns = "box",
             int id = 0, const ros::Duration lifetime = ros::Duration(),
             const std_msgs::ColorRGBA color = msg::createColorMsg(0.5, 0.5, 0.5, 0.5),
             bool destroy = false, bool frame_locked = false);

void drawSphere(ros::Publisher &pub, const shapes::Sphere &shape, const char *ns = "sphere",
             int id = 0, const ros::Duration lifetime = ros::Duration(),
             const std_msgs::ColorRGBA color = msg::createColorMsg(0.5, 0.5, 0.5, 0.5),
             bool destroy = false, bool frame_locked = false);

void drawArrow(ros::Publisher &pub, const shapes::Arrow &shape, const char *ns = "arrow",
             int id = 0, const ros::Duration lifetime = ros::Duration(),
             const std_msgs::ColorRGBA color = msg::createColorMsg(0.5, 0.5, 0.5, 0.5),
             bool destroy = false, bool frame_locked = false);

void drawMesh (ros::Publisher &pub, const shapes::Mesh &shape, const char *ns = "mesh",
               int id = 0, const ros::Duration lifetime = ros::Duration(),
               const std_msgs::ColorRGBA color = msg::createColorMsg(0.5, 0.5, 0.5, 0.5),
               bool destroy = false, bool frame_locked = false);

} // namespace object_manipulator

#endif
