// process_pcl_v2.cpp: wsn, March, 2015
// example code to show some point-cloud processing, including interaction with Rviz selected points


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
//#include <Transform.h>
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
//geometry_msgs::Point computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
void computeRsqd(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, Eigen::Vector3f centroid, std::vector<float> &rsqd_vec);
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,std::vector<int>iselect);
void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud);
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices);
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices,PointCloud<pcl::PointXYZ>::Ptr outputCloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect; //(new PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_display_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// PointXYZRGB would be colorized
//tf::transform transform;
Eigen::Matrix3f g_R_transform;
// wakes up when a new "selected Points" message arrives
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelect(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *pclSelect);
    ROS_INFO("Select %d * %d points", pclSelect->width, pclSelect->height);
    //ROS_INFO("frame id is: %s",cloud->header.frame_id);
    cout<<"header frame: "<<cloud->header.frame_id<<endl;
    int npts = pclSelect->width*pclSelect->height;
    
    geometry_msgs::Point centroid;
    Eigen::Vector3f centroidEvec3f;
    
    
    centroidEvec3f = computeCentroid(pclSelect); // compute the centroid of this point cloud (selected patch)
  
    std::vector<float> rsqd_vec;
    computeRsqd(pclSelect,centroidEvec3f,rsqd_vec);
    ROS_INFO("computing rsqd vec: ");
    float variance=0.0;
    for (int i=0;i<rsqd_vec.size();i++) {
        variance+=rsqd_vec[i];
        //cout<<rsqd_vec[i]<<", ";      
    }
    cout<<endl;
    variance/=( (float) npts);

    // now, eliminate any outliers; actually, keep only points withing 1 std;
    std::vector<int> iselect_all;
    std::vector<int> iselect_filtered;
    for (int i=0;i<npts;i++) {
        iselect_all.push_back(i); //not choosey--pick all the points
        if (rsqd_vec[i]<variance) {
            iselect_filtered.push_back(i); // choosey: retain only points within 1 std dev
        }
    }
    //ROS_INFO("selected indices: ");
    //for (int i=0;i<iselect_filtered.size();i++) {
    //    cout<<iselect_filtered[i]<<", ";
    //}
    //cout<<endl;
    cout<<"npts = "<<npts<<endl;
    cout<<"npts selected: "<<iselect_filtered.size()<<endl;
    cout<<"variance = "<<variance<<endl;
    cout<<"std_dev: "<<sqrt(variance)<<endl;   
    cout<<"unfiltered centroid: "<<centroidEvec3f.transpose()<<endl;
    centroidEvec3f = computeCentroid(pclSelect,iselect_filtered);
    cout<<"refined centroid:    "<<centroidEvec3f.transpose()<<endl;
    
    NormalEstimation<PointXYZ, Normal> n; // object to compute the normal to a set of points

    float curvature; 
    Eigen::Vector4f plane_params;
    
    n.computePointNormal (*pclSelect, iselect_all, plane_params, curvature); // compute normal of the selected points
    std::cout<<"plane_params:           "<<plane_params.transpose()<<std::endl;  
    n.computePointNormal (*pclSelect, iselect_filtered, plane_params, curvature); // redo for the filtered points
    std::cout<<"plane_params, filtered: "<<plane_params.transpose()<<std::endl;   
    
    Eigen::Vector3f  plane_normal;
    Eigen::Vector3f  x_dir;
    for (int i=0;i<3;i++) plane_normal[i] = plane_params[i]; //for selected patch, get plane normal from point-cloud processing result
    
    x_dir<<1,0,0; // keep x-axis the same...nominally
    x_dir = x_dir - plane_normal*(plane_normal.dot(x_dir)); // force x-dir to be orthogonal to z-dir
    x_dir/= x_dir.norm(); // want this to be unit length as well
    //populate g_R_transform with the direction vectors of the plane frame, with respect to the kinect frame
    g_R_transform.col(0) = x_dir;
    g_R_transform.col(2)= plane_normal; // want the z-axis to be the plane normal    
    g_R_transform.col(1) = g_R_transform.col(2).cross(g_R_transform.col(0)); //make y-axis consistent right-hand triad
    // use the following to transform kinect points into the plane frame; could do translation as well, but not done here
    Eigen::Matrix3f R_transpose = g_R_transform.transpose();
    
    transform_cloud(pclSelect,R_transpose,g_cloud_out); // try rotating the selected points  
    // g_cloud_out from above operation should now all have the same z-value, if these points are coplanar
    // let's compute the normal of these points to check:
    n.computePointNormal (*g_cloud_out, iselect_all, plane_params, curvature);
    //we should find that the computed normal is (0,0,1)
    std::cout<<"plane_params, rotated: "<<plane_params.transpose()<<std::endl;    
 
    double z_plane_nom = -plane_params[3]; // distance of plane from sensor origin--same as distance measured along plane normal
    // after rotating the points to align with the plane of the selected patch, all z-values should be approximately the same,
    // = z_plane_nom
    double z_eps = 0.01; // choose a tolerance for plane inclusion +/- z; 1cm??
     vector<int> indices_z_eps; // vector that will hold indices of points that qualify as close enough to the selected plane
    filter_cloud_z(g_cloud_out,z_plane_nom,z_eps,indices_z_eps);   //list all point indices that seem to lie on indicated plane
    copy_cloud(pclSelect, indices_z_eps,g_display_cloud); // create an output point cloud, which is selected from points of input cloud
    // the above only extracts qualifying points from the selected patch--primarily useful for testing
    
    //OK...let's try transforming the ENTIRE point cloud:
    transform_cloud(g_pclKinect,R_transpose,g_cloud_out); // rotate the entire point cloud
    // g_cloud_out is now expressed in the frame of the selected plane;
    // let's extract all of the points (i.e., name the indices of these points) for which the z value corresponds to the chosen plane
    filter_cloud_z(g_cloud_out,z_plane_nom,z_eps,indices_z_eps);
    // point indices of interest are in indices_z_eps; use this to extract this subset from the parent cloud to create a new cloud
    copy_cloud(g_pclKinect, indices_z_eps,g_display_cloud); //g_display_cloud is being published regularly by "main"

}


/*
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
 * */

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

//2nd version: no selection vector provided, --> use all the points
Eigen::Vector3f computeCentroid(PointCloud<pcl::PointXYZ>::Ptr pcl_cloud) {
    Eigen::Vector3f centroid;
    centroid << 0,0,0;

    int size = pcl_cloud->width * pcl_cloud->height;
    std::cout<<"frame: "<<pcl_cloud->header.frame_id<<std::endl;
    for(size_t i = 0; i != size; ++i) {  
        centroid += pcl_cloud->points[i].getVector3fMap();
    }
    if (size>0) {
       centroid/= ((float) size);
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

void transform_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Matrix3f R_xform, PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    // use the global pointcloud:
    outputCloud->header   = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width    = inputCloud->width;
    outputCloud->height   = inputCloud->height;
    int npts = inputCloud->points.size();
        cout<<"transforming npts = "<<npts<<endl;
    outputCloud->points.resize(npts);
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt= R_xform * inputCloud->points[i].getVector3fMap ();
        //cout<<"transformed pt: "<<pt.transpose()<<endl;
      outputCloud->points[i].getVector3fMap () = pt; //R_xform * inputCloud->points[i].getVector3fMap ();
    }
} 

//make a new point cloud, extracted from inputCloud using indices in "indices"
void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, vector<int> &indices,PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
    int npts = indices.size(); //how many points to extract?
        outputCloud->header   = inputCloud->header;
    outputCloud->is_dense = inputCloud->is_dense;
    outputCloud->width    = npts;
    outputCloud->height   = 1;

    cout<<"copying cloud w/ npts ="<<npts<<endl;
    outputCloud->points.resize(npts);
    for (int i = 0; i < npts; ++i) {
      outputCloud->points[i].getVector3fMap () = inputCloud->points[indices[i]].getVector3fMap ();
    }
}

//given a cloud, identify which points are within z_eps of z_nom; put these point indices in "indices"
void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, vector<int> &indices) {
    int npts = inputCloud->points.size();
    Eigen::Vector3f pt;
    indices.clear();
    double dz;
    int ans;
    for (int i = 0; i < npts; ++i) {  
        pt = inputCloud->points[i].getVector3fMap();
        //cout<<"pt: "<<pt.transpose()<<endl;
        dz = pt[2] - z_nom;
        if (fabs(dz)<z_eps) {
            indices.push_back(i);
            //cout<<"dz = "<<dz<<"; saving this point...enter 1 to continue: ";
            //cin>>ans;
        }
    }
    int n_extracted = indices.size();
    cout<<" number of points in range = "<<n_extracted<<endl;
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
    //ROS_INFO("kinectCB %d * %d points", pclKinect->width, pclKinect->height);
 
}

int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    // Subscribers
    ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);
    ros::Subscriber selectedPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/plane_model", 1);
    sensor_msgs::PointCloud2 ros_cloud;
    
    
    while (ros::ok()) {
        pcl::toROSMsg(*g_display_cloud, ros_cloud);
        pubCloud.publish(ros_cloud);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}