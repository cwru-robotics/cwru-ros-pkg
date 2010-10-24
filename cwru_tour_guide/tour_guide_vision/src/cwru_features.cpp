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
  this->r_value=0;
}

RawFeature::~RawFeature(){}

void RawFeature::print(){
 printf("%f %f %f %f %f %f %f %f\n",x,y,theta,pixel.x,pixel.y,ray.x,ray.y,ray.z); 
}

//assigns the input r value and calculates the feature position given the camera offset
cv::Point3d RawFeature::calcPosition(double r, const cv::Mat &camera_frame_offset){
  
  cv::Mat theta_rotation=cv::Mat::eye(3,3,CV_64F);
  theta_rotation.at<double>(0,0)=cos(theta);
  theta_rotation.at<double>(0,1)=-sin(theta);
  theta_rotation.at<double>(1,0)=sin(theta);
  theta_rotation.at<double>(1,1)=cos(theta);
  
  
  cv::Mat camera_pos=cv::Mat::zeros(3,1,CV_64F);
  camera_pos.at<double>(0,0)=x;
  camera_pos.at<double>(1,0)=y;
  camera_pos=theta_rotation*camera_frame_offset+camera_pos;
  
  r_value=r;
  
  lastCalcedPosition.x=lastCalcedRay.x*r_value+camera_pos.at<double>(0,0);
  lastCalcedPosition.y=lastCalcedRay.y*r_value+camera_pos.at<double>(1,0);
  lastCalcedPosition.z=lastCalcedRay.z*r_value+camera_pos.at<double>(2,0);

  return lastCalcedPosition;
}
 
 
 //calculates the unit vector from camera to robot in map frame   
cv::Point3d RawFeature::calcRay(double yaw, double pitch, double roll){

 // printf("ypr %f %f %f\n",yaw,pitch,roll);
  //convert ray to a matrix
  
  //this puts the ray in the robots coordinate frame
  cv::Mat ray_mat(3,1,CV_64F);
  ray_mat.at<double>(0,0)=this->ray.z;
  ray_mat.at<double>(1,0)=-this->ray.x;
  ray_mat.at<double>(2,0)=-this->ray.y;

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
  
    //make rotation matrix
  cv::Mat theta_rotation=cv::Mat::eye(3,3,CV_64F);
  theta_rotation.at<double>(0,0)=cos(this->theta);
  theta_rotation.at<double>(0,1)=-sin(this->theta);
  theta_rotation.at<double>(1,0)=sin(this->theta);
  theta_rotation.at<double>(1,1)=cos(this->theta);
  
  cv::Mat camera_extrinsics_wrtrobot=yaw_mat*(pitch_mat*roll_mat);
  
  /*   
   printf("theta mat \n%f %f %f\n%f %f %f\n%f %f %f\n",
	 theta_rotation.at<double>(0,0),theta_rotation.at<double>(0,1),theta_rotation.at<double>(0,2),
	 theta_rotation.at<double>(1,0),theta_rotation.at<double>(1,1),theta_rotation.at<double>(1,2),
	 theta_rotation.at<double>(2,0),theta_rotation.at<double>(2,1),theta_rotation.at<double>(2,2));  	 
  
   printf("rotation mat \n%f %f %f\n%f %f %f\n%f %f %f\n",
	 camera_extrinsics_wrtrobot.at<double>(0,0),camera_extrinsics_wrtrobot.at<double>(0,1),camera_extrinsics_wrtrobot.at<double>(0,2),
	 camera_extrinsics_wrtrobot.at<double>(1,0),camera_extrinsics_wrtrobot.at<double>(1,1),camera_extrinsics_wrtrobot.at<double>(1,2),
	 camera_extrinsics_wrtrobot.at<double>(2,0),camera_extrinsics_wrtrobot.at<double>(2,1),camera_extrinsics_wrtrobot.at<double>(2,2));
*/
 // printf("ray mat \n%f %f %f\n",ray_mat.at<double>(0,0),ray_mat.at<double>(1,0),ray_mat.at<double>(2,0));
	   

  //this is the part to edit   
  cv::Point3d output_ray;
  cv::Mat output_ray_mat = (theta_rotation * (camera_extrinsics_wrtrobot* ray_mat));

  output_ray.x=output_ray_mat.at<double>(0,0);
  output_ray.y=output_ray_mat.at<double>(1,0);
  output_ray.z=output_ray_mat.at<double>(2,0);
  
// printf("output ray %f %f %f\n",output_ray.x, output_ray.y, output_ray.z);
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
cv::Point3d FeatureManager::calc_least_squares_position(double yaw, double pitch, double roll,const cv::Mat &camera_frame_offset){
//we cannot calculate the least squares of 1 feature
  if(raw_features.size()<2){
    cv::Point3d bad=cv::Point3d(-1,-1,-1);
    return bad;
  }
  
//  printf("starting function\n");
  //create V matrix
  cv::Mat V=cv::Mat::zeros((raw_features.size()-1)*3,raw_features.size(),CV_64F);
  
  //create C matrix
  cv::Mat C=cv::Mat::zeros((raw_features.size()-1)*3,1,CV_64F);
  
 // printf("features %d",raw_features.size());
  
  //printf("created matricies\n");
  //for the first raw feature we have
  //calculate the ray and put the pose and ray in the matrixes
  raw_features[0].calcRay( yaw, pitch, roll);
  
//  printf("calculated first ray\n");


  cv::Mat theta_rotation=cv::Mat::eye(3,3,CV_64F);
  theta_rotation.at<double>(0,0)=cos(raw_features[0].theta);
  theta_rotation.at<double>(0,1)=-sin(raw_features[0].theta);
  theta_rotation.at<double>(1,0)=sin(raw_features[0].theta);
  theta_rotation.at<double>(1,1)=cos(raw_features[0].theta);


  cv::Mat p1=cv::Mat::zeros(3,1,CV_64F);
  p1.at<double>(0,0)=raw_features[0].x;
  p1.at<double>(1,0)=raw_features[0].y;
  p1=theta_rotation*camera_frame_offset+p1;
  
  cv::Mat p2=cv::Mat::zeros(3,1,CV_64F);
  
  for(int i=0;i<raw_features.size()-1;i++){
    V.at<double>(i*3,0)=raw_features[0].lastCalcedRay.x;
    V.at<double>(i*3+1,0)=raw_features[0].lastCalcedRay.y;
    V.at<double>(i*3+2,0)=raw_features[0].lastCalcedRay.z;
    
    theta_rotation.at<double>(0,0)=cos(raw_features[i+1].theta);
    theta_rotation.at<double>(0,1)=-sin(raw_features[i+1].theta);
    theta_rotation.at<double>(1,0)=sin(raw_features[i+1].theta);
    theta_rotation.at<double>(1,1)=cos(raw_features[i+1].theta);
    
    p2.at<double>(0,0)=raw_features[i+1].x;
    p2.at<double>(1,0)=raw_features[i+1].y;
    
    cv::Mat p3=(theta_rotation*camera_frame_offset+p2)-p1;
    
    //assign all position values
    C.at<double>(i*3,0)=p3.at<double>(0,0);
    C.at<double>(i*3+1,0)=p3.at<double>(1,0);
    C.at<double>(i*3+2,0)=0;
    
  }
  
//  printf("calculated all rays\n");
//printf("rows %d, cols %d\n",C.rows,C.cols);
  
  //for every other raw feature we have
  for(int i=0;i<raw_features.size()-1;i++){
    //calculate the ray in the map frame of that feature
    raw_features[i+1].calcRay( yaw, pitch, roll);
    
    //assign the value into the least squares matrix correctly
    V.at<double>(i*3,i+1)=-raw_features[i+1].lastCalcedRay.x;
    V.at<double>(i*3+1,i+1)=-raw_features[i+1].lastCalcedRay.y;
    V.at<double>(i*3+2,i+1)=-raw_features[i+1].lastCalcedRay.z;
  }
  /*
  for(int i=0;i<V.rows;i++){
      for(int j=0;j<V.cols;j++){
	printf("%f ", V.at<double>(i,j));
      }
      printf("\n");
  }
  printf("\n\n");
   for(int i=0;i<C.rows;i++){
      for(int j=0;j<C.cols;j++){
	printf("%f ", C.at<double>(i,j));
      }
      printf("\n");
  }
*/
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
    
    raw_features[i].calcPosition(r_mat.at<double>(i,0),camera_frame_offset);
    //printf("%f, ",raw_features[i].r_value);
    
  //  printf("position %f %f %f\n",raw_features[i].lastCalcedPosition.x,raw_features[i].lastCalcedPosition.y,raw_features[i].lastCalcedPosition.z);
    
    mean.x+=raw_features[i].lastCalcedPosition.x;
    mean.y+=raw_features[i].lastCalcedPosition.y;
    mean.z+=raw_features[i].lastCalcedPosition.z;
    
  }
//  printf("\n");
  mean.x/=raw_features.size();
  mean.y/=raw_features.size();
  mean.z/=raw_features.size();
  
//  printf("mean %f %f %f\n",mean.x, mean.y, mean.z);
//  printf("calculated f positions\n");
  
  cv::Point3d error;
  error.x=0;
  error.y=0;
  error.z=0;
  
  cv::Point3d error_vect=cv::Point3d(0,0,0);
  
  for(int i=0;i<raw_features.size();i++){
    double temp_sqr;
    
    temp_sqr=mean.x-raw_features[i].lastCalcedPosition.x;
    error.x=temp_sqr*temp_sqr;
    
    temp_sqr=mean.y-raw_features[i].lastCalcedPosition.y;
    error.y=temp_sqr*temp_sqr;
    
    temp_sqr=mean.z-raw_features[i].lastCalcedPosition.z;
    error.z=temp_sqr*temp_sqr;
    
    error_vect.x+=error.x;
    error_vect.y+=error.y;
    error_vect.z+=error.z;
    
    //variance is the sum of the squares of the distance from each point and the mean
  }
  
  error_vect.x/=raw_features.size()-1;
  error_vect.y/=raw_features.size()-1;
  error_vect.z/=raw_features.size()-1;
 
  printf("feature errors %f %f %f\n",error_vect.x,error_vect.y,error_vect.z);  
  printf("calculated error with size %d\n",raw_features.size());
  return error_vect;
}

void FeatureManager::add(const RawFeature &add_me){
  raw_features.push_back(add_me);
}
int FeatureManager::numFeatures(){
  return raw_features.size();
}


