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

#include <cwru_srv/simple_int_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package

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

void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps); 

void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params);

// a bunch of pointcloud holders, all global

//pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclKinect(new PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_from_disk (new pcl::PointCloud<pcl::PointXYZ>); //this one is read from PCD file on disk
pcl::PointCloud<pcl::PointXYZ>::Ptr g_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // holder for processed point clouds
pcl::PointCloud<pcl::PointXYZ>::Ptr g_display_cloud(new pcl::PointCloud<pcl::PointXYZ>); // this cloud gets published--viewable in rviz
pcl::PointCloud<pcl::PointXYZ>::Ptr g_pclSelect(new pcl::PointCloud<pcl::PointXYZ>); // holds published points, per Rviz tool
// PointXYZRGB would be colorized

Eigen::Matrix3f g_R_transform; // a matrix useful for rotating the data

const int IDENTIFY_PLANE=0;
const int FIND_ON_TABLE=1;

int g_pcl_process_mode = 0;

bool modeService(cwru_srv::simple_int_service_messageRequest& request, cwru_srv::simple_int_service_messageResponse& response)
{
    ROS_INFO("mode select service callback activated");
    response.resp = true; // boring, but valid response info
    g_pcl_process_mode = request.req;
    cout<<"Mode set to: "<<g_pcl_process_mode<<endl;
    return true;
}

// wakes up when a new "selected Points" message arrives
void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud) {

    pcl::fromROSMsg(*cloud, *g_pclSelect);
    ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", g_pclSelect->width, g_pclSelect->height);
    //ROS_INFO("frame id is: %s",cloud->header.frame_id);
    cout<<"header frame: "<<cloud->header.frame_id<<endl;
    int npts = g_pclSelect->width*g_pclSelect->height;
    std::vector<int> iselect_filtered; //indices of patch that do not contain outliers
    std::vector<int> indices_of_plane; //indices of patch that do not contain outliers    
    Eigen::Vector4f plane_params;
    
    Eigen::Vector3f centroidEvec3f;
    
    process_patch(iselect_filtered, centroidEvec3f, plane_params); // operate on selected points to remove outliers and
      //find centroid and plane params
    
    switch(g_pcl_process_mode)  { // what we do here depends on our mode; mode is settable via a service
            case IDENTIFY_PLANE:
                ROS_INFO("MODE 0: identifying plane based on selection...");
                find_plane(plane_params,indices_of_plane);
                
                break;
                
        case FIND_ON_TABLE:
            ROS_INFO("filtering for objects on most recently defined plane: not implemented yet");
            
            break;
        default:
            ROS_WARN("this mode is not implemented");
            
    }   
}

// process patch: filter selected points to remove outliers;
// then compute the centroid and the plane parameters of the filtered points
// return these values in centroidEvec3f and plane_params
void process_patch(std::vector<int> &iselect_filtered, Eigen::Vector3f &centroidEvec3f, Eigen::Vector4f &plane_params) {
    ROS_INFO("PROCESSING THE PATCH: ");
    int npts = g_pclSelect->width*g_pclSelect->height;
    centroidEvec3f = computeCentroid(g_pclSelect); // compute the centroid of this point cloud (selected patch)
    std::vector<float> rsqd_vec;
    computeRsqd(g_pclSelect,centroidEvec3f,rsqd_vec);
    //ROS_INFO("computing rsqd vec: ");
    float variance=0.0;
    for (int i=0;i<rsqd_vec.size();i++) {
        variance+=rsqd_vec[i];
        //cout<<rsqd_vec[i]<<", ";      
    }
    cout<<endl;
    variance/=( (float) npts);

    // now, eliminate any outliers; actually, keep only points withing 1 std;
     for (int i=0;i<npts;i++) {
        if (rsqd_vec[i]<variance) {
            iselect_filtered.push_back(i); // choosey: retain only points within 1 std dev
        }
    }   
    cout<<"npts = "<<npts<<endl;
    cout<<"npts of filtered patch: "<<iselect_filtered.size()<<endl;
    cout<<"variance = "<<variance<<endl;
    cout<<"std_dev: "<<sqrt(variance)<<endl;       
    centroidEvec3f = computeCentroid(g_pclSelect,iselect_filtered);
    cout<<"refined centroid:    "<<centroidEvec3f.transpose()<<endl;

    NormalEstimation<PointXYZ, Normal> n; // object to compute the normal to a set of points

    float curvature;     
    
    n.computePointNormal (*g_pclSelect, iselect_filtered, plane_params, curvature); // find plane params for filtered patch
    std::cout<<"plane_params, filtered patch: "<<plane_params.transpose()<<std::endl;       
    
}
// this function operates on the global cloud pointer g_cloud_from_disk;
// g_cloud_out contains a cloud rotated s.t. identified plane has normal (0,0,1), 
//  indices_z_eps contain the indices of the points on the identified plane;
//  g_display_cloud is a reduced version of g_cloud_from_disk, w/ only the planar points (expressed in original frame)
void find_plane(Eigen::Vector4f plane_params, std::vector<int> &indices_z_eps) {
    float curvature;  
    std::vector<int> iselect_all;
    NormalEstimation<PointXYZ, Normal> n; // object to compute the normal to a set of points 
    
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
    
    //transform_cloud(g_pclSelect,R_transpose,g_cloud_out); // try rotating the selected points  
    // g_cloud_out from above operation should now all have the same z-value, if these points are coplanar
    // let's compute the normal of these points to check:
    //iselect_all.clear();  // this is silly... should have a version of computeNormal that does not require a list of indices,
      //if we want ALL points to be considered
     //for (int i=0;i<g_cloud_out->points.size();i++) {
      //      iselect_all.push_back(i); // choosey: retain only points within 1 std dev
      //  }
   
    //n.computePointNormal (*g_cloud_out, iselect_all, plane_params, curvature);
    //we should find that the computed normal is (0,0,1)
    //std::cout<<"plane_params, rotated: "<<plane_params.transpose()<<std::endl;    
 
    double z_plane_nom = -plane_params[3]; // distance of plane from sensor origin--same as distance measured along plane normal
    // after rotating the points to align with the plane of the selected patch, all z-values should be approximately the same,
    // = z_plane_nom
    double z_eps = 0.01; // choose a tolerance for plane inclusion +/- z; 1cm??
    // vector<int> indices_z_eps; // vector that will hold indices of points that qualify as close enough to the selected plane
    //filter_cloud_z(g_cloud_out,z_plane_nom,z_eps,indices_z_eps);   //list all point indices that seem to lie on indicated plane
    //copy_cloud(g_pclSelect, indices_z_eps,g_display_cloud); // create an output point cloud, which is selected from points of input cloud
    // the above only extracts qualifying points from the selected patch--primarily useful for testing
    
    //OK...let's try transforming the ENTIRE point cloud:
    transform_cloud(g_cloud_from_disk,R_transpose,g_cloud_out); // rotate the entire point cloud
    // g_cloud_out is now expressed in the frame of the selected plane;
    // let's extract all of the points (i.e., name the indices of these points) for which the z value corresponds to the chosen plane
    filter_cloud_z(g_cloud_out,z_plane_nom,z_eps,indices_z_eps);
    // point indices of interest are in indices_z_eps; use this to extract this subset from the parent cloud to create a new cloud
    copy_cloud(g_cloud_from_disk, indices_z_eps,g_display_cloud); //g_display_cloud is being published regularly by "main"

}


// given a point cloud, compute the centroid. Mostly useful for small patches, since centroid of the whole cloud is not too useful
// will operate only on the points listed by index in iselect
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

// compute the distance-squared of each point from the provided centroid
// presumably useful for outlier removal filtering
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

// given an input cloud, rotate ALL points using matrix R_xform, and put the result in outputCloud
// This should be generalized for an affine transform (translation plus rotation)
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



int main(int argc, char** argv) {
    // Do some initialization here
    ros::init(argc, argv, "process_pcl");
    ros::NodeHandle nh;
    ros::Rate rate(2);
    // Subscribers
    //ros::Subscriber getPCLPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/kinect/depth/points", 1, kinectCB);
    ros::Subscriber selectedPoints = nh.subscribe<sensor_msgs::PointCloud2> ("/selected_points", 1, selectCB);
    
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/plane_model", 1);
    ros::Publisher pubPcdCloud = nh.advertise<sensor_msgs::PointCloud2> ("/kinect_pointcloud", 1);   
    
   ros::ServiceServer service = nh.advertiseService("process_mode", modeService);
   
    sensor_msgs::PointCloud2 ros_cloud;

  
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *g_cloud_from_disk) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << g_cloud_from_disk->width * g_cloud_from_disk->height
            << " data points from test_pcd.pcd  "<< std::endl;
  
  g_cloud_from_disk->header.frame_id = "world"; //kinect_pc_frame"; // why is this necessary?  Did PCD delete the reference frame id?
    
    while (ros::ok()) {
        pcl::toROSMsg(*g_cloud_from_disk, ros_cloud);        
        pcl::toROSMsg(*g_display_cloud, ros_cloud);
        pubPcdCloud.publish(g_cloud_from_disk);
        pubCloud.publish(ros_cloud);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}