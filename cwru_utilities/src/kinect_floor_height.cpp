#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
//  ROS_INFO("Got a point cloud message");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*msg, cloud);
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud(cloud.makeShared());
  seg.segment(inliers, coefficients);
  double roll, pitch, yaw;
  tf::Quaternion q = tf::Quaternion(coefficients.values[0]/coefficients.values[3], coefficients.values[1]/coefficients.values[3], coefficients.values[2]/coefficients.values[3], 1.0);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double camera_pitch = (3.14159/2.0)+pitch;
  double camera_roll = 0.0;
  double camera_yaw = roll;
  double camera_height = -coefficients.values[3];
  static tf::TransformBroadcaster tf_br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, camera_height) );
  transform.setRotation( tf::createQuaternionFromRPY(camera_roll, camera_pitch, 0.0) );
  tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/camera_link"));
  tf::Transform floor_tf;
  floor_tf.setOrigin( tf::Vector3(0.0, 0.0, coefficients.values[3]) );
  floor_tf.setRotation( q );
  tf_br.sendTransform(tf::StampedTransform(floor_tf, ros::Time::now(),"/camera_depth_frame", "/floor_normal"));
  ROS_INFO("Got a PCL Message\nHeight:: %.6f m\nPitch: %.6f rad (%.1f deg)\nRoll: %.6f rad (%.1f deg)\nYaw: %.6f (%.1f deg)\n\n",
          camera_height, camera_pitch, (camera_pitch / 3.14159 * 180.0), camera_roll, (camera_roll / 3.14159 * 180.0), camera_yaw, (camera_yaw / 3.14159 * 180.0) );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ROS_INFO("Subscribing to /camera/depth/points...");
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
  ROS_INFO("Subscribed to /camera/depth/points.");
  ros::spin();
}
