// process_pcd_save.cpp: wsn, April, 2015
// example code to acquire a pointcloud from a topic, and save a snapshot to disk
// as a PCD file.

#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>



#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
//#include <pcl/common/impl/centroid.hpp>

//PointCloud::Ptr pclCloud(new PointCloud); // Holds the whole pcl cloud
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);

bool got_cloud = false;
bool saved_file = false;

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *g_pclKinect);
    //ROS_INFO("kinectCB %d * %d points", pclKinect->width, pclKinect->height);
        got_cloud=true; //cue to "main" to save snapshot
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    // Subscribers
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);

    
    
    while (ros::ok()) {       
        if (got_cloud) {
            if (!saved_file) {
                pcl::io::savePCDFileASCII ("test_pcd.pcd", *g_pclKinect);
                std::cerr << "Saved " << g_pclKinect->points.size () << " data points to test_pcd.pcd." << std::endl;
                std::cout<<"frame: "<<g_pclKinect->header.frame_id<<std::endl;
                saved_file=true; // mark that we have saved a file--don't need to repeat this; just do it once                
                return 0; // or bail out now
            }
        }
        else 
            cout<<"waiting for got_cloud..."<<endl;
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}