#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <laser_geometry/laser_geometry.h>
#include <boost/shared_ptr.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  boost::shared_ptr<tf::TransformListener> tf_listener_ptr;
  boost::shared_ptr<tf::TransformBroadcaster> tf_br_ptr;
  boost::shared_ptr<ros::Subscriber> sub_ptr;  
  sensor_msgs::PointCloud2 laser_cloud_msg;
  boost::shared_ptr< pcl::PointCloud<pcl::PointXYZ> > laser_cloud_ptr;
  
  void floor_pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
	ROS_DEBUG("Got a point cloud message with %d points", msg->height * msg->row_step);
    
    //Transform point cloud into the world (/base_link frame)
    pcl::fromROSMsg (*msg, cloud_in);
    ROS_DEBUG("Converted a point cloud message with %d points.",cloud_in.size());
    pcl_ros::transformPointCloud("/base_link", cloud_in, cloud, *tf_listener_ptr);
    ROS_DEBUG("Transformed point cloud message with %d points", cloud.size());
    seg.setInputCloud(cloud.makeShared());
    //Segment it
    seg.segment(inliers, coefficients);
	
	//Find the angle of the floor
    //Create a TF from the segmentation coefficients
    Eigen::AngleAxis<float> aa = Eigen::AngleAxis<float>(acos(coefficients.values[0]),
                                Eigen::Vector3f(0,-coefficients.values[2],
                                                coefficients.values[1]));
    Eigen::Quaternion<float> q = Eigen::Quaternion<float>(aa);
    geometry_msgs::TransformStamped tx;
    tx.transform.rotation.x = q.x();
    tx.transform.rotation.y = q.y();
    tx.transform.rotation.z = q.z();
    tx.transform.rotation.w = q.w();
    tx.transform.translation.z = - coefficients.values[3] / coefficients.values[2];
    tx.child_frame_id = "/floor";
    tx.header.frame_id = "/base_link";
    tx.header.stamp = ros::Time::now();
    tf_br_ptr->sendTransform(tx);
    ROS_INFO("Published floor transform");
  }
  
  void init_segmentation(){
    ROS_INFO("Configuring SAC segmentation...");
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    ROS_INFO("Waiting for PointCloud2 Messages...");
  }

  void wall_laser_callback(const sensor_msgs::LaserScanConstPtr& msg){
    laser_geometry::LaserProjection projector_;

    ROS_DEBUG("Got a laser message.");
    //Convert laserscan to pointcloud
    projector_.transformLaserScanToPointCloud("/base_link", *msg, laser_cloud_msg, *tf_listener_ptr, -1.0, laser_geometry::channel_option::Default);
    ROS_DEBUG("Transformed and converted laser message to point cloud message.");
    laser_cloud_ptr = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg (laser_cloud_msg, *laser_cloud_ptr);
    ROS_DEBUG("Converted laser point cloud message to point cloud.");
    //Fit a line to the pointcloud
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (laser_cloud_ptr));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    Eigen::VectorXf line_coefficients;
    ransac.getModelCoefficients(line_coefficients);
    ROS_DEBUG("Got model coefficients for laser line.");
    //Make a TF from the wall line    
    Eigen::AngleAxis<float> aa = Eigen::AngleAxis<float>(0.0,
                                Eigen::Vector3f(line_coefficients[3],line_coefficients[4],
                                                line_coefficients[5]));
    Eigen::Quaternion<float> q = Eigen::Quaternion<float>(aa);
    geometry_msgs::TransformStamped tx;
    tx.transform.rotation.x = q.x();
    tx.transform.rotation.y = q.y();
    tx.transform.rotation.z = q.z();
    tx.transform.rotation.w = q.w();
    tx.transform.translation.x = line_coefficients[0];
    tx.transform.translation.y = 0.0;
    tx.transform.translation.z = line_coefficients[2];
    
    tx.child_frame_id = "/laser_wall";
    tx.header.frame_id = "/base_link";
    tx.header.stamp = ros::Time::now();
    tf_br_ptr->sendTransform(tx);
    ROS_INFO("Published laser wall transform");
  }

  void wall_pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
	ROS_DEBUG("Got a point cloud message");
    
    //Transform point cloud into the world (/base_link frame)
    pcl::fromROSMsg (*msg, cloud_in);
    ROS_DEBUG("Converted point cloud message to point cloud.");
    pcl_ros::transformPointCloud("/base_link", cloud_in, cloud, *tf_listener_ptr);
    ROS_DEBUG("Tranformed point cloud.");
    seg.setInputCloud(cloud.makeShared());
    //Segment it
    seg.segment(inliers, coefficients);
	
	//Find the angle of the floor
    //Create a TF from the segmentation coefficients
    Eigen::AngleAxisf aa(acos(coefficients.values[0]),
                                Eigen::Vector3f(0,-coefficients.values[2],
                                                coefficients.values[1]));
    Eigen::Quaternion<float> q(aa);
    geometry_msgs::TransformStamped tx;
    tx.transform.rotation.x = q.x();
    tx.transform.rotation.y = q.y();
    tx.transform.rotation.z = q.z();
    tx.transform.rotation.w = q.w();
    tx.transform.translation.x = -coefficients.values[3] / coefficients.values[0];
    tx.child_frame_id = "/wall";
    tx.header.frame_id = "/base_link";
    tx.header.stamp = ros::Time::now();
    tf_br_ptr->sendTransform(tx);
    ROS_INFO("Published wall transform");
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_calibrator");
  ros::NodeHandle nh;
  ROS_INFO("Setting up TF Broadcaster and listener...");
  tf_br_ptr = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
  tf_listener_ptr = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
  try{
    tf_listener_ptr->waitForTransform("/camera_rgb_frame","/base_link", ros::Time(0),ros::Duration(10.0) );
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }
  init_segmentation();
  ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, floor_pcl_callback); 
  //ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, wall_pcl_callback); 
  //ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_laser1_scan", 1, wall_laser_callback);
  ros::spin();
}
