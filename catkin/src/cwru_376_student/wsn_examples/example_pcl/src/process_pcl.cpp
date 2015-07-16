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
geometry_msgs::Point computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec);
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,std::vector<int>iselect);

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect; //(new PointCloud<pcl::PointXYZ>);

// wakes up when a new "selected Points" message arrives
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelect(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pclSelect);
    ROS_INFO("Select %d * %d points", pclSelect->width, pclSelect->height);
    //ROS_INFO("frame id is: %s",cloud->header.frame_id);
    cout<<"header frame: "<<cloud->header.frame_id<<endl;
    int npts = pclSelect->width*pclSelect->height;

    //    Eigen::Vector4f centroidV4f;
    //pcl::compute3dCentroid(pclSelect,centroidV4f);
    //ROS_INFO("try pcl fnc...");
    //cout<<"centroidV4f: "<<centroidV4f.transpose()<<endl;
    
    geometry_msgs::Point centroid;
    Eigen::Vector3f centroidEvec3f;
    centroid = computeCentroid(pclSelect);
    centroidEvec3f(0) = centroid.x;
    centroidEvec3f(1) = centroid.y;
    centroidEvec3f(2) = centroid.z;    
    std::vector<float> rsqd_vec;
    computeRsqd(pclSelect,centroidEvec3f,rsqd_vec);
    ROS_INFO("rsqd vec: ");
    float variance=0.0;
    for (int i=0;i<rsqd_vec.size();i++) {
        variance+=rsqd_vec[i];
        cout<<rsqd_vec[i]<<", ";      
    }
    cout<<endl;
    variance/=( (float) npts);

    //cout<<rsqd_vec[0];
    // now, eliminate any outliers; actually, keep only points withing 1 std;
    std::vector<int> iselect;
    for (int i=0;i<npts;i++) {
        if (rsqd_vec[i]<variance) {
            iselect.push_back(i);
        }
    }
    ROS_INFO("select indices: ");
    for (int i=0;i<iselect.size();i++) {
        cout<<iselect[i]<<", ";
    }
    cout<<endl;
    cout<<"npts = "<<npts<<endl;
    cout<<"npts selected: "<<iselect.size()<<endl;
     cout<<"variance = "<<variance<<endl;
    cout<<"std_dev: "<<sqrt(variance)<<endl;   
    cout<<"unfiltered centroid: "<<centroidEvec3f.transpose()<<endl;
    centroidEvec3f = computeCentroid(pclSelect,iselect);
    cout<<"refined centroid:    "<<centroidEvec3f.transpose()<<endl;
    
    //ROS_INFO("Got the centroid");
    //ROS_INFO("Position: x = %f, y = %f, z = %f", centroid.x, centroid.y, centroid.z);
    
    geometry_msgs::PointStamped centroidStamped;
    centroidStamped.header = cloud->header; //"utorso"; //targetFrame;
    centroidStamped.point = centroid;
    //centroidStampPub.publish(centroidStamped);
      NormalEstimation<PointXYZ, Normal> n;
        // computePointNormal (indices, Vector)
      //g_pclKinect = &cloud3;
      //n.computePointNormal (cloud3, indices, plane_params, curvature);
    float curvature; 
    Eigen::Vector4f plane_params;
    std::vector<int> indices;
    for (int i=0;i<npts;i++) indices.push_back(i);      
    n.computePointNormal (*pclSelect, indices, plane_params, curvature);
    std::cout<<"plane_params:           "<<plane_params.transpose()<<std::endl;  
    n.computePointNormal (*pclSelect, iselect, plane_params, curvature);
    std::cout<<"plane_params, filtered: "<<plane_params.transpose()<<std::endl;        
}



geometry_msgs::Point computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
    geometry_msgs::Point centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;
    uint32_t size = pcl_cloud->width * pcl_cloud->height;
    std::cout<<"frame: "<<pcl_cloud->header.frame_id<<std::endl;
    for(size_t i = 0; i != size; ++i) {
        centroid.x += pcl_cloud->points[i].x;
        centroid.y += pcl_cloud->points[i].y;
        centroid.z += pcl_cloud->points[i].z;
    }
    centroid.x = centroid.x / size;
    centroid.y = centroid.y / size;
    centroid.z = centroid.z / size;
    return centroid;
}

Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,std::vector<int>iselect) {
    Eigen::Vector3f centroid;
    centroid << 0,0,0;
    int nselect = iselect.size();
    for (int i=0;i<nselect;i++) {
        centroid += pcl_cloud->points[iselect[i]].getVector3fMap();
    }
    if (nselect>0) {
       centroid/= ((float) nselect);
    }
    return centroid;
}

void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,Eigen::Vector3f centroid, std::vector<float> &rsqd_vec) {
    Eigen::Vector3f evec3f;
    int npts = pcl_cloud->points.size();
    rsqd_vec.clear();
    rsqd_vec.resize(npts);
    for (int i=0; i<npts ;i++) {
     evec3f = pcl_cloud->points[i].getVector3fMap();
     evec3f-= centroid;
     rsqd_vec[i] = evec3f.dot(evec3f);
    }
}

void printPoints() {
    float x, y, z;
    int ans;
    std::cout<<"frame: "<<g_pclKinect->header.frame_id<<std::endl;
    int width = g_pclKinect->width;
    int height = g_pclKinect->height;
    double xmin, xmax, ymin, ymax, zmin, zmax;
    zmin = 1000;
    zmax = -1000;
    xmin = 1000;
    xmax = -1000;
    ymin = 1000;
    ymax = -1000;
    uint32_t size = width * height;
    for (int iheight = 0; iheight < height; iheight++) {
        for (int iwidth = 0; iwidth < width; iwidth++) {
            int i = iheight * width + iwidth;
            x = g_pclKinect->points[i].x;
            y = g_pclKinect->points[i].y;
            z = g_pclKinect->points[i].z;
            cout << "x,y,z = " << x << ", " << y << ", " << z << endl;
            if (y>ymax) ymax = y;
            if (y<ymin) ymin = y;
            if (x>xmax) xmax = x;
            if (x<xmin) xmin = x;
            if (z>zmax) zmax = z;
            if (z<zmin) zmin = z;
        }
        cout<<"row "<<iheight<<endl;
        cout<<"xmin, xmax ="<<xmin<<", "<<xmax<<endl;
        cout<<"ymin, ymax ="<<ymin<<", "<<ymax<<endl;
        cout<<"zmin, zmax ="<<zmin<<", "<<zmax<<endl;        
        //cout << "enter 1: ";
        //cin >> ans;
    }

}

void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect(new PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pclKinect);
    g_pclKinect = pclKinect;
    ROS_INFO("kinectCB %d * %d points", pclKinect->width, pclKinect->height);
        geometry_msgs::Point centroid;
    centroid = computeCentroid(pclKinect);
    ROS_INFO("Got the centroid");
    ROS_INFO("Position: x = %f, y = %f, z = %f", centroid.x, centroid.y, centroid.z);
    int ipt = rand()%(pclKinect->width*pclKinect->height-10);
    
    //printPoints();
    //computePointNormal (const pcl::PointCloud< PointInT > &cloud, const std::vector< int > &indices, Eigen::Vector4f &plane_parameters, float &curvature)
    float curvature; 
    Eigen::Vector4f plane_params;
    std::vector<int> indices;
    for (int i=0;i<10;i++) indices.push_back(i+ipt);
    //note: template<class PointT> void pcl::computePointNormal(const pcl::PointCloud<PointT>&, const std::vector<int>&, Eigen::Vector4f&, float&)
    //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    //no known conversion for argument 1 from ‘pcl::PointCloud<pcl::PointXYZ>::Ptr {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >}’ to ‘const pcl::PointCloud<pcl::PointXYZ>&’
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    
    //pcl::PointCloud<PointXYZ> cloud3;
    
    //ne.setInputCloud(new_cloud); 
    //ne.computePointNormal (cloud3, indices, plane_params, curvature);
    
      NormalEstimation<PointXYZ, Normal> n;
        // computePointNormal (indices, Vector)
      //g_pclKinect = &cloud3;
      //n.computePointNormal (cloud3, indices, plane_params, curvature);
      n.computePointNormal (*g_pclKinect, indices, plane_params, curvature);
      std::cout<<"plane_params: "<<plane_params.transpose()<<std::endl;
      
    //no matching function for call to ‘pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::computePointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr&, std::vector<int>&, Eigen::Vector4f&, double&)’
   //no known conversion for argument 1 from ‘pcl::PointCloud<pcl::PointXYZ>::Ptr {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >}’ to ‘const pcl::PointCloud<pcl::PointXYZ>&’
   //no known conversion for argument 1 from ‘pcl::PointCloud<pcl::PointXYZ>::Ptr* {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >*}’ to ‘const pcl::PointCloud<pcl::PointXYZ>&’
    //ne.computePointNormal(new_cloud,indices,plane_params,curvature);
    //ne.computePointNormal(&temp_cloud,indices,plane_params,curvature);
    //pcl::computePointNormal(new_cloud,indices,plane_params,curvature);
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    // Subscribers
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);
    ros::Subscriber selectedPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}