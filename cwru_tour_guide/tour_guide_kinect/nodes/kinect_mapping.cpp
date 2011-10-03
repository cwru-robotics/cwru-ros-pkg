#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <fstream>
#include <octomap/OcTreeLabeled.h>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

void kinectPointsCallback(octomap::OctomapROS<octomap::OcTree>* oct, const sensor_msgs::PointCloud2::ConstPtr& msg, tf::TransformListener& tfl)
{

  ROS_INFO("Capturing Kinect data!");

  sensor_msgs::PointCloud2 mapCloud;
  
  geometry_msgs::PointStamped dummyOrigin;

  dummyOrigin.header.frame_id = "base_link";
  dummyOrigin.header.stamp = ros::Time::now();
  dummyOrigin.point.x = 0;
  dummyOrigin.point.y = 0;
  dummyOrigin.point.z = 0.9;

  geometry_msgs::PointStamped mapOrigin;

  sensor_msgs::PointCloud2 cloud = *msg;

  cloud.header.frame_id = "base_link";

  pcl::PointXYZ origin;
  origin.x = mapOrigin.point.x;
  origin.y = mapOrigin.point.y;
  origin.z = mapOrigin.point.z;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;


  try{

    pcl_ros::transformPointCloud("map", cloud, mapCloud, tfl);
    tfl.transformPoint("map", dummyOrigin, mapOrigin);
    pcl::fromROSMsg(mapCloud, pcl_cloud);

    ROS_INFO("success!");

    oct->insertScan(pcl_cloud, origin, -1, true);
    ROS_INFO("actual success");
  }
  catch(tf::TransformException& ex){
    ROS_INFO("caught exception, ignored it");
    ROS_INFO(ex.what());
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kinect_listener");

  ros::NodeHandle n;

  tf::TransformListener tfl(ros::Duration(1));

  tfl.waitForTransform("map", "baselink", ros::Time::now(), ros::Duration(10));
  
  octomap::OctomapROS<octomap::OcTree>* octree = new octomap::OctomapROS<octomap::OcTree>::OctomapROS(0.1);

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,  boost::bind(kinectPointsCallback, octree, _1, boost::ref(tfl)));

  ros::Rate r(1);

  while(ros::ok())
  {
    ROS_INFO("spinning");
    ros::spinOnce();
    r.sleep();
  }
  
  octree->octree.writeBinary("octreeBin.bt");

  return 0;
}
