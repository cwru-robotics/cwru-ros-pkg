#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
    plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (msg->points));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane, 0.1);
  ransac.computeModel();

  ransac.getInliers(inliers);
  Eigen::VectorXf floor_coefficients;
  plane.computeModelCoefficients(&inliers, &floor_coefficients);
  printf ("Kinect is %d meters from the ground", floor_coefficients[3]);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
}
