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

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "object_manipulator/tools/msg_helpers.h"
#include "object_manipulator/tools/shape_tools.h"

namespace object_manipulator {

void drawCylinder (ros::Publisher &pub, const shapes::Cylinder &shape, const char *ns,
              int id, const ros::Duration lifetime, const std_msgs::ColorRGBA color, bool destroy, bool frame_locked)
{
    visualization_msgs::Marker marker;
    marker.header = shape.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER; // CUBE, SPHERE, ARROW, CYLINDER
    marker.action = destroy?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
    tf::poseTFToMsg(shape.frame, marker.pose);
    tf::vector3TFToMsg(shape.dims, marker.scale);
    marker.color = color;
    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    pub.publish(marker);
}

void drawBox (ros::Publisher &pub, const shapes::Box &shape, const char *ns,
              int id, const ros::Duration lifetime, const std_msgs::ColorRGBA color, bool destroy, bool frame_locked)
{
    visualization_msgs::Marker marker;
    marker.header = shape.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE; // CUBE, SPHERE, ARROW, CYLINDER
    marker.action = destroy?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
    tf::poseTFToMsg(shape.frame, marker.pose);
    tf::vector3TFToMsg(shape.dims, marker.scale);
    marker.color = color;
    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    pub.publish(marker);
}

void drawSphere (ros::Publisher &pub, const shapes::Sphere &shape, const char *ns,
              int id, const ros::Duration lifetime, const std_msgs::ColorRGBA color, bool destroy, bool frame_locked)
{
    visualization_msgs::Marker marker;
    marker.header = shape.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE; // CUBE, SPHERE, ARROW, CYLINDER
    marker.action = destroy?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
    tf::poseTFToMsg(shape.frame, marker.pose);
    tf::vector3TFToMsg(shape.dims, marker.scale);
    marker.color = color;
    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    pub.publish(marker);
}

void drawArrow (ros::Publisher &pub, const shapes::Arrow &shape, const char *ns,
              int id, const ros::Duration lifetime, const std_msgs::ColorRGBA color, bool destroy, bool frame_locked)
{
    visualization_msgs::Marker marker;
    marker.header = shape.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW; // CUBE, SPHERE, ARROW, CYLINDER
    marker.action = destroy?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
    tf::poseTFToMsg(shape.frame, marker.pose);
    tf::vector3TFToMsg(shape.dims, marker.scale);
    marker.color = color;
    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    pub.publish(marker);
}

void drawMesh (ros::Publisher &pub, const shapes::Mesh &shape, const char *ns,
              int id, const ros::Duration lifetime, const std_msgs::ColorRGBA color, bool destroy, bool frame_locked)
{
    visualization_msgs::Marker marker;
    marker.header = shape.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE; // CUBE, SPHERE, ARROW, CYLINDER
    marker.mesh_resource = shape.mesh_resource;
    marker.mesh_use_embedded_materials = shape.use_embedded_materials;
    marker.action = destroy?((int32_t)visualization_msgs::Marker::DELETE):((int32_t)visualization_msgs::Marker::ADD);
    tf::poseTFToMsg(shape.frame, marker.pose);
    tf::vector3TFToMsg(shape.dims, marker.scale);
    marker.color = color;
    marker.lifetime = lifetime;
    marker.frame_locked = frame_locked;
    pub.publish(marker);
}

} // namespace object_manipulator
