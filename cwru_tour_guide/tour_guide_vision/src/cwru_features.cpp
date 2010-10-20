#include "cwru_features.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <stdio.h>

RawFeature::RawFeature(){}
RawFeature::RawFeature(double x, double y, double theta, cv::Point2d pixel,cv::Point3d ray){
  this->x=x;
  this->y=y;
  this->theta=theta;
  this->pixel=pixel;
  this->ray=ray;
}

RawFeature::~RawFeature(){}

void RawFeature::print(){
 printf("%f %f %f %f %f %f %f %f\n",x,y,theta,pixel.x,pixel.y,ray.x,ray.y,ray.z); 
}


cv::Point3d RawFeature::calcRay(const cv::Mat& camera2robot_axis,double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset){

 // printf("ypr %f %f %f\n",yaw,pitch,roll);
  //convert ray to a matrix
  cv::Mat ray_mat(3,1,CV_64F);
  ray_mat.at<double>(0,0)=this->ray.x;
  ray_mat.at<double>(1,0)=this->ray.y;
  ray_mat.at<double>(2,0)=this->ray.z;
  
  //make pose translation matrix
  //pose=x,y,0
  cv::Mat robot_pose(3,1,CV_64F);
  robot_pose.at<double>(0,0)=this->x;
  robot_pose.at<double>(1,0)=this->y;
  robot_pose.at<double>(2,0)=0;
  
  //make rotation matrix
  cv::Mat theta_rotation=cv::Mat::eye(3,3,CV_64F);
  theta_rotation.at<double>(0,0)=cos(this->theta);
  theta_rotation.at<double>(0,1)=-sin(this->theta);
  theta_rotation.at<double>(1,0)=sin(this->theta);
  theta_rotation.at<double>(1,1)=cos(this->theta);
  
  
  //make extrinsics matrix
  cv::Mat yaw_mat=cv::Mat::eye(3,3,CV_64F);
  yaw_mat.at<double>(0,0)=cos(yaw);
  yaw_mat.at<double>(0,1)=-sin(yaw);
  yaw_mat.at<double>(1,0)=sin(yaw);
  yaw_mat.at<double>(1,1)=cos(yaw);
  
  
  cv::Mat pitch_mat=cv::Mat::eye(3,3,CV_64F);
  pitch_mat.at<double>(0,0)=cos(pitch);
  pitch_mat.at<double>(0,2)=sin(pitch);
  pitch_mat.at<double>(2,0)=-sin(pitch);
  pitch_mat.at<double>(2,2)=cos(pitch);
  
  
  cv::Mat roll_mat=cv::Mat::eye(3,3,CV_64F);
  roll_mat.at<double>(1,1)=cos(roll);
  roll_mat.at<double>(1,2)=-sin(roll);
  roll_mat.at<double>(2,1)=sin(roll);
  roll_mat.at<double>(2,2)=cos(roll);
  
  cv::Mat camera_extrinsics_wrtrobot=yaw_mat*(pitch_mat*roll_mat);
  
  /*
  printf("pose mat \n%f %f %f\n",
	 robot_pose.at<double>(0,0),robot_pose.at<double>(1,0),robot_pose.at<double>(2,0));
	 
    
   printf("theta mat \n%f %f %f\n%f %f %f\n%f %f %f\n",
	 theta_rotation.at<double>(0,0),theta_rotation.at<double>(0,1),theta_rotation.at<double>(0,2),
	 theta_rotation.at<double>(1,0),theta_rotation.at<double>(1,1),theta_rotation.at<double>(1,2),
	 theta_rotation.at<double>(2,0),theta_rotation.at<double>(2,1),theta_rotation.at<double>(2,2));
  
  
	 
  printf("camera_frame_offset mat \n%f %f %f\n",
	 camera_frame_offset.at<double>(0,0),camera_frame_offset.at<double>(1,0),camera_frame_offset.at<double>(2,0));
	 
  
   printf("rotation mat \n%f %f %f\n%f %f %f\n%f %f %f\n",
	 camera_extrinsics_wrtrobot.at<double>(0,0),camera_extrinsics_wrtrobot.at<double>(0,1),camera_extrinsics_wrtrobot.at<double>(0,2),
	 camera_extrinsics_wrtrobot.at<double>(1,0),camera_extrinsics_wrtrobot.at<double>(1,1),camera_extrinsics_wrtrobot.at<double>(1,2),
	 camera_extrinsics_wrtrobot.at<double>(2,0),camera_extrinsics_wrtrobot.at<double>(2,1),camera_extrinsics_wrtrobot.at<double>(2,2));
    
	 
   printf("camera2robot_axis mat \n%f %f %f\n%f %f %f\n%f %f %f\n",
	 camera2robot_axis.at<double>(0,0),camera2robot_axis.at<double>(0,1),camera2robot_axis.at<double>(0,2),
	 camera2robot_axis.at<double>(1,0),camera2robot_axis.at<double>(1,1),camera2robot_axis.at<double>(1,2),
	 camera2robot_axis.at<double>(2,0),camera2robot_axis.at<double>(2,1),camera2robot_axis.at<double>(2,2));

  printf("ray mat \n%f %f %f\n",
	 ray_mat.at<double>(0,0),ray_mat.at<double>(1,0),ray_mat.at<double>(2,0));
	   */

   
  cv::Point3d output_ray;
  cv::Mat output_ray_mat = robot_pose+ (theta_rotation * (camera_frame_offset+ (camera_extrinsics_wrtrobot* (camera2robot_axis*ray_mat))));
  
  
  output_ray.x=output_ray_mat.at<double>(0,0);
  output_ray.y=output_ray_mat.at<double>(1,0);
  output_ray.z=output_ray_mat.at<double>(2,0);
  
  //printf("output ray %f %f %f\n",output_ray.x, output_ray.y, output_ray.z);
  lastCalcedRay=output_ray;
  return output_ray;
}



FeatureManager::FeatureManager(){
  currentMean.x=0;
  currentMean.y=0;
  currentMean.z=0;
}

FeatureManager::~FeatureManager(){
  
}


void FeatureManager::print(){
  for(int i=0;i<raw_features.size();i++){
    raw_features[i].print();
  }
}
void FeatureManager::merge(const FeatureManager & merger){
  //merge all feature sighting of this other feature with all feature sightings we have
  
}

//return the error of this least squares
double FeatureManager::calc_least_squares_position(const cv::Mat& camera2robot_axis,double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset){
//we cannot calculate the least squares of 1 feature
  if(raw_features.size()<2){
    return -1;
  }
  
//  printf("starting function\n");
  //create V matrix
  cv::Mat V=cv::Mat::zeros((raw_features.size()-1)*3,raw_features.size(),CV_64F);
  
  //create C matrix
  cv::Mat C=cv::Mat::zeros((raw_features.size()-1)*3,1,CV_64F);
  
  
  //printf("created matricies\n");
  //for the first raw feature we have
  //calculate the ray and put the pose and ray in the matrixes
  raw_features[0].calcRay(camera2robot_axis, yaw, pitch, roll,camera_frame_offset);
  
//  printf("calculated first ray\n");
  for(int i=0;i<raw_features.size()-1;i++){
    V.at<double>(i*3,0)=raw_features[0].lastCalcedRay.x;
    V.at<double>(i*3+1,0)=raw_features[0].lastCalcedRay.y;
    V.at<double>(i*3+2,0)=raw_features[0].lastCalcedRay.z;
    
    //assign all position values
    C.at<double>(i*3,0)=raw_features[i+1].x-raw_features[0].x;
    C.at<double>(i*3+1,0)=raw_features[i+1].y-raw_features[0].y;
    C.at<double>(i*3+2,0)=0;
    
  }
  
//  printf("calculated all rays\n");
  
  //for every other raw feature we have
  for(int i=0;i<raw_features.size()-1;i++){
    //calculate the ray in the map frame of that feature
    raw_features[i+1].calcRay(camera2robot_axis, yaw, pitch, roll,camera_frame_offset);
    
    //assign the value into the least squares matrix correctly
    V.at<double>(i*3,i+1)=-raw_features[i+1].lastCalcedRay.x;
    V.at<double>(i*3+1,i+1)=-raw_features[i+1].lastCalcedRay.y;
    V.at<double>(i*3+2,i+1)=-raw_features[i+1].lastCalcedRay.z;
  }
  
  
//  printf("filled matrix\n");
  
//  printf("the matrix is inverting %d\n",raw_features.size()); 
  
  cv::Mat invertedV=V.inv(cv::DECOMP_SVD);
  
//  printf("inverted\n");
  cv::Mat r_mat=invertedV*C;
  
//  printf("calculated r matrix\n");
  
  cv::Point3d mean;
  mean.x=0;
  mean.y=0;
  mean.z=0;
  
  for(int i=0;i<raw_features.size();i++){
    raw_features[i].r_value=r_mat.at<double>(i,0);
    mean.x+=raw_features[i].lastCalcedRay.x*raw_features[i].r_value;
    mean.y+=raw_features[i].lastCalcedRay.y*raw_features[i].r_value;
    mean.z+=raw_features[i].lastCalcedRay.z*raw_features[i].r_value;
    
//    printf("%f, ",raw_features[i].r_value);
  }
//  printf("\n");
  mean.x/=raw_features.size();
  mean.y/=raw_features.size();
  mean.z/=raw_features.size();
  
  
//  printf("calculated f positions\n");
  
  cv::Point3d error;
  error.x=0;
  error.y=0;
  error.z=0;
  
  double mag_error=0;
  cv::Point3d error_vect=cv::Point3d(0,0,0);
  for(int i=0;i<raw_features.size();i++){
    double temp_sqr;
    temp_sqr=mean.x-raw_features[i].lastCalcedRay.x*raw_features[i].r_value;
    error.x=temp_sqr*temp_sqr;
    temp_sqr=mean.y-raw_features[i].lastCalcedRay.y*raw_features[i].r_value;
    error.y=temp_sqr*temp_sqr;
    temp_sqr=mean.z-raw_features[i].lastCalcedRay.z*raw_features[i].r_value;
    error.z=temp_sqr*temp_sqr;
    
    
    error_vect.x+=error.x;
    error_vect.y+=error.y;
    error_vect.z+=error.z;
    
    //variance is the sum of the squares of the distance from each point and the mean
    mag_error+=error.x+error.y+error.z;
  }
  
 printf("feature errors %f %f %f\n",error_vect.x,error_vect.y,error_vect.z);
  
  cv::Point3d this_error;

  if(mag_error>1){
    printf("feature mean %f %f %f  of %d      \n",mean.x,mean.y,mean.z,raw_features.size());
    for(int i=0;i<raw_features.size();i++){
      this_error.x=mean.x-raw_features[i].lastCalcedRay.x*raw_features[i].r_value;
      this_error.y=mean.y-raw_features[i].lastCalcedRay.y*raw_features[i].r_value;
      this_error.z=mean.z-raw_features[i].lastCalcedRay.z*raw_features[i].r_value;
   //   printf("feature %f %f %f ;\n",raw_features[i].lastCalcedRay.x*raw_features[i].r_value,raw_features[i].lastCalcedRay.y*raw_features[i].r_value,raw_features[i].lastCalcedRay.z*raw_features[i].r_value);

      if(sqrt(this_error.x*this_error.x+this_error.y*this_error.y+this_error.z*this_error.z)>sqrt(mag_error)){
	printf("large error of %f\n",sqrt(this_error.x*this_error.x+this_error.y*this_error.y+this_error.z*this_error.z));
      }
    }
  }
  
  printf("calculated error with size %d\n",raw_features.size());
  return mag_error;
}

void FeatureManager::add(const RawFeature &add_me){
  raw_features.push_back(add_me);
}
int FeatureManager::numFeatures(){
  return raw_features.size();
}


