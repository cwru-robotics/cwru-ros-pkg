/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Eric Perko, Jesse Fish; 2012, Edward Venator
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
 *   * Neither the name of Case Western Reserve University nor the names of its
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
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <string>
#include <list>

std::list<geometry_msgs::Point32> points;
std::string frame_id;
bool recieved_map;
bool updated_map;

void mapOccupancyGridCallback(const  nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	points.clear();
	recieved_map=true;
	updated_map=true;
	frame_id=msg->header.frame_id;
	geometry_msgs::Point32 p;
	for(unsigned int m=0;m<msg->info.height;m++){
		for(unsigned int n=0;n<msg->info.width;n++){
			//currently 100 is the only value used for obsticles in the map
			if(msg->data[m*msg->info.width+n]==100){
				p.x=n*msg->info.resolution+msg->info.origin.position.x;
				p.y=m*msg->info.resolution+msg->info.origin.position.y;
				points.push_back(p);
			}
		}
	}
}


int main(int argc, char *argv[]){
	//boolean switch for if we have ever recieved a map
	recieved_map=false;
	//boolean switch that triggers every time the map is updated
	updated_map=false;


	//intensity of all points in the cloud
	double intensity=100;
	//rate in Hz the sensor outputs its data
	int sensor_rate=5;
	//height of the sensor in meters
	double height=8.;
	//name of the channel the points are in, for the cloud
	std::string channel_name="intensity";

	//name of the tf frame to put the point cloud in so that the origin of the point cloud will be within the rolling window for the local costmap
	std::string target_frame="base_link";

	//start node
	ros::init(argc, argv, "map_as_sensor");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh_("~"); 

	// parameters for the node
	priv_nh_.param("intensity", intensity,100.);
	priv_nh_.param("sensor_rate",sensor_rate, 1);
	priv_nh_.param("channel_name",channel_name, std::string("intensity"));
	priv_nh_.param("height",height, 1.9);
	priv_nh_.param("target_frame", target_frame, std::string("base_link"));

	ros::Subscriber sub = n.subscribe("input_map", 1, &mapOccupancyGridCallback);

	ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("map_cloud",1);

	sensor_msgs::PointCloud * sensor_points=NULL;
	//continuously publish the point cloud at a set rate
	ros::Rate r(sensor_rate);

	ros::Rate sleep_rate(1.0);
	while(ros::ok() && !recieved_map) {
	   ROS_INFO("Waiting to receive a map before looking up the transform information"); 
	   ros::spinOnce();
	   sleep_rate.sleep();
	}
	tf::TransformListener listener;
	listener.waitForTransform(target_frame, frame_id, ros::Time::now(), ros::Duration(10));
	sensor_msgs::PointCloud temp_out;

	while(n.ok()){
		if(recieved_map){
			if(updated_map){
				updated_map=false;

				if(sensor_points!=NULL){ delete(sensor_points);}

				sensor_points=NULL;
				sensor_points=new sensor_msgs::PointCloud;

				sensor_points->points.resize(points.size());

				//sensor_points->set_channels_size(1);
				sensor_points->channels.resize(1);
				sensor_points->channels[0].name = channel_name;
				sensor_points->channels[0].values.resize(points.size());

				sensor_points->header.frame_id =frame_id;
				//for everything in list points, set the x and y value to the right stuff, and set intensity to intensity
				int point_number=0;
				for(std::list<geometry_msgs::Point32>::const_iterator iterator = points.begin(), end = points.end(); iterator != end; ++iterator){
					sensor_points->points[point_number].x = (*iterator).x;
					sensor_points->points[point_number].y = (*iterator).y;
					sensor_points->points[point_number].z = height;
					sensor_points->channels[0].values[point_number] = (float)intensity;
					++point_number;
				}
			}

			ros::Time latest_transform_time = ros::Time::now();
			listener.getLatestCommonTime(frame_id, target_frame, latest_transform_time, NULL);
			sensor_points->header.stamp = latest_transform_time;
			listener.transformPointCloud(target_frame, *sensor_points, temp_out);
			cloud_pub.publish(temp_out);
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

