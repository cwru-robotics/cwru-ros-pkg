#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/CvBridge.h>
#include <string>
#include <stdio.h>
#include <vector>
#include "cwru_features.h"


class FeatureTracker {
  public:
    FeatureTracker();
    ~FeatureTracker();
  private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    
    void track_features(geometry_msgs::PoseStamped mapPose);
    
    cv::Point3d calc_error(int min_features,double dyaw,double dpitch, double droll);
    
    
    IplImage * image_rect;
    IplImage * last_image;
    
    
    IplImage * output_image;
    
    //variables for finding initial features in the image
    IplImage * eigImage;
    IplImage * tempImage;
    int num_features;
    CvPoint2D32f * features;
    double quality_level;
    double min_distance;
    int block_size;
    
    
    //variables for tracking features with optical flow
    IplImage * pyrA;
    IplImage * pyrB;
    int last_feature_count;
    CvPoint2D32f * featuresB;
    char * last_features_status;
    float * track_error;
    int win_size;
    
    
    int * last_feature_id;
    int * current_feature_id;
    
    cv::Mat camera_frame_offset;
    cv::Mat camera_extrinsics_wrtrobot;
//    cv::Mat camera2robot_axis;
    
    double yaw;
    double pitch;
    double roll;
    
    
    double yalpha;
    double palpha;
    double ralpha;
    
    sensor_msgs::CvBridge bridge;
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;  
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber cam_subscriber;
    image_transport::Publisher analyzed_pub_;
    
    image_geometry::PinholeCameraModel cam_model;
    std::vector<FeatureManager> featureList;
    int feature_number;
    tf::TransformListener tf_listener_;
    int pic_number;
    
    
    int pixel_tracking_margin;
};



FeatureTracker::FeatureTracker() : it_(nh_), priv_nh_("~"){
  priv_nh_.param("num_features",num_features, 15);
  priv_nh_.param("min_distance", min_distance, 10.0);
  priv_nh_.param("quality_level", quality_level, 0.1);
  priv_nh_.param("block_size", block_size, 7);
  
  priv_nh_.param("win_size", win_size, 10);
  
  
  
   yalpha= .025/2;
   palpha= .025/2;
   ralpha= .025/2;
   
  
  //offset of the camera frame, start at 0,0,0 and teak x and y eventually
  camera_frame_offset=cv::Mat::zeros(3,1,CV_64F);
  
  camera_frame_offset.at<double>(0,0)=.06;
  //rough estimation of the extrinsic calibration for the camera
  //should be a parameter
  yaw=0.8 * 3.14159/180;
  pitch=-34.38 * 3.14159/180;
  roll=-3.87 * 3.14159/180;
  
  
  feature_number=0;
  pic_number=0;
  image_rect=NULL;
  
  output_image=NULL;
  
  eigImage=NULL;
  tempImage=NULL;
  
  pyrA=NULL;
  pyrB=NULL;
  
  features=new CvPoint2D32f[num_features];
  featuresB=new CvPoint2D32f[num_features];
  last_features_status=new char[num_features];
  current_feature_id=new int[num_features];
  last_feature_id=new int[num_features];
  track_error=new float[num_features];
  last_feature_count=0;
  
  for(int i=0;i<num_features;i++){
    last_features_status[i]=0;
    last_feature_id[i]=-1;
    current_feature_id[i]=-1;
  }
  
  
  cam_subscriber= it_.subscribeCamera("image_rect_color", 1, &FeatureTracker::image_callback, this);
  
  analyzed_pub_= it_.advertise("feature_image", 1);
  
  pixel_tracking_margin=1;
}

FeatureTracker::~FeatureTracker(){
  delete [] features;
  delete [] last_features_status;
  delete [] track_error;
  delete [] featuresB;
  delete [] last_feature_id;
  delete [] current_feature_id;
  //delete images
  
}


void FeatureTracker::track_features(geometry_msgs::PoseStamped mapPose){
  //set the initial number of features to the max number we want to find
  int feature_count=num_features;
  printf("pose %f %f %f\n",mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation));
  int edge_pixels=5;
  
  //check if there were features from the last image to keep tracking
  if(last_feature_count>0){
    //if there were call cvCalcOpticalFlowPyrLK();
    //find matches between last good features and current image features
    //    store matches in featuresB
    cvCalcOpticalFlowPyrLK(last_image,image_rect,pyrA,pyrB,features,featuresB, last_feature_count,cvSize(win_size,win_size) ,4,last_features_status,track_error, cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,.3),0);
  }
  
  printf("got image flow\n");
  //    assign last_feature_id values for matched features and set the non matched spots to -1
  
  //find new features and subpixel them
  
  //I SHOULD ADD THE IMAGE FLOW VALUES AS FEATURES NOW BEFORE FINDING NEW FEATURES
  
  //find all good features
  cvGoodFeaturesToTrack(image_rect, eigImage, tempImage, features, &feature_count, quality_level, min_distance, NULL, block_size);
  
  //subpixel good features
  cvFindCornerSubPix(image_rect,features,feature_count,cvSize(win_size,win_size),cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
  
  
  printf("subpixeled image\n");
  
  //for all the features in features B, find their matches in the newly found features
  //add all the matches to their correct featuremanager, for the non matching, make a new
  //feature manager and add them to it
  
  //for all features by now we need their ray and the robot pose at that location
  //draw dots on image where features are
  
  
  
  //set the feature ids to a control value
  
  for(int i=0;i<num_features;i++){
    current_feature_id[i]=-1;
  }
  
  for(int i=0;i<last_feature_count;i++){
    //for the previously found features in list b
    if(last_features_status[i]>0){
      for(int j=0;j<feature_count;j++){
	//for every feature found in this image
	
	//determine if the two overlap in a meaningful way
	int xdiff=featuresB[i].x-features[j].x;
	int ydiff=featuresB[i].y-features[j].y;
	//if the pixels are within some margin of eachother
	if(sqrt(xdiff*xdiff + ydiff*ydiff)<pixel_tracking_margin){
	  //if they do set the current id for j to the id of i
	  current_feature_id[j]=last_feature_id[i];
	  printf("feature found %d %d",last_feature_id[i],i);
	}
      }
    }
  }
  
  printf("assigned IDs image\n");
  
  
  for(int i=0;i<feature_count;i++){
    
    printf("looping\n");
    if(current_feature_id[i]>=0){
    printf("prev feature match\n");
      //if we matched a previous feature
      //add our new feature to the previous features list
      cv::Point3d tempRay;
      cv::Point2d tempPoint=cv::Point2d(features[i]);
      cam_model.projectPixelTo3dRay(tempPoint,tempRay);
      
      if(tempPoint.x> edge_pixels && tempPoint.x < last_image->width- edge_pixels &&
	tempPoint.y> edge_pixels && tempPoint.y<last_image->height- edge_pixels){
	featureList[current_feature_id[i]].add(RawFeature(mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation), tempPoint,tempRay));
      }else{
	current_feature_id[i]=-1;
      }
      
    }else{
    printf("new feature \n");
      
      cv::Point3d tempRay;
      cv::Point2d tempPoint=cv::Point2d(features[i]);
      cam_model.projectPixelTo3dRay(tempPoint,tempRay);
      if(tempPoint.x> edge_pixels && tempPoint.x < last_image->width- edge_pixels &&
	tempPoint.y> edge_pixels && tempPoint.y<last_image->height- edge_pixels){
	printf("new good feature \n");
	//if we didn't
	//create a new feature group in the list
	current_feature_id[i]=feature_number;
	//add the new feature to the feature list
	featureList.push_back(FeatureManager());

	featureList[feature_number].add(RawFeature(mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation), tempPoint,tempRay));
	++feature_number;
      }
    }
  }
   
//  printf("features: ");
  for(int i=0;i<num_features;i++){
    if(i<feature_count){
     last_feature_id[i]=current_feature_id[i];
    }
    else{
      last_feature_id[i]=-1;
    }
 //   printf(" %d ",current_feature_id[i]);
  }
  printf("\n");
  
  
  last_feature_count=feature_count;
  
}

cv::Point3d FeatureTracker::calc_error(int min_features, double dyaw,double dpitch, double droll){
  cv::Point3d error_sum=cv::Point3d(0,0,0);
  
  for(int i=0;i<featureList.size();i++){
    if(min_features<featureList[i].numFeatures()){
      printf("\n\n\nfeature %d\n",i);
//      featureList[i].print();
      cv::Point3d error=featureList[i].calc_least_squares_position(yaw+dyaw, pitch+dpitch, roll+droll, camera_frame_offset);
      error_sum+=error;
      
    }
  }
  return error_sum;
}

void FeatureTracker::image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  //need pose data for each picture, need to publish a camera pose
  ros::Time acquisition_time = msg->header.stamp;
  geometry_msgs::PoseStamped basePose;
  geometry_msgs::PoseStamped mapPose;
  basePose.pose.orientation.w=1.0;
  ros::Duration timeout(3);
  basePose.header.frame_id="/base_link";
  mapPose.header.frame_id="/map";
  
  try {
    tf_listener_.waitForTransform("/camera_1_link", "/map", acquisition_time, timeout);
    tf_listener_.transformPose("/map", acquisition_time,basePose,"/camera_1_link",mapPose);
    printf("pose #%d %f %f %f\n",pic_number++,mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation));
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("[map_maker] TF exception:\n%s", ex.what());
    printf("[map_maker] TF exception:\n%s", ex.what());
    return;
  }
  cam_model.fromCameraInfo(info_msg);
  
  
  
  
  // printf("callback called\n");
  try
  {
    // if you want to work with color images, change from mono8 to bgr8
    if(image_rect==NULL){
      image_rect = cvCloneImage(bridge.imgMsgToCv(msg, "mono8"));
      last_image= cvCloneImage(bridge.imgMsgToCv(msg, "mono8"));
      pyrA=cvCreateImage(cvSize(last_image->width+8,last_image->height/3.0), IPL_DEPTH_32F, 1);
      pyrB=cvCloneImage(pyrA);
      //  printf("cloned image\n");
    }
    else{
      //save the last image
      cvCopy(image_rect,last_image);
      cvCopy(bridge.imgMsgToCv(msg, "mono8"),image_rect);
      // printf("copied image\n");
    }
    if(output_image==NULL){
      output_image =cvCloneImage(image_rect);
    }
    if(eigImage==NULL){
      eigImage =cvCloneImage(image_rect);
    }
    if(tempImage==NULL){
      tempImage =cvCloneImage(image_rect);
    }
  }
  catch (sensor_msgs::CvBridgeException& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    return;
  }
  
  if(image_rect!=NULL) {
    cvCopy(image_rect,output_image);
    
    printf("got image\n");
    
    track_features(mapPose);
    
    //draw features on the image
    for(int i=0;i<last_feature_count;i++){
      CvPoint center=cvPoint((int)features[i].x,(int)features[i].y);
      cvCircle(output_image,center,10,cvScalar(150),2);
      
      char strbuf [10];
      
      int n=sprintf(strbuf,"%d",current_feature_id[ i] );
      std::string text=std::string(strbuf,n);
      
      CvFont font;
      
      cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1,1);
      
      cvPutText(output_image,text.c_str(),cvPoint(center.x,center.y+20),&font,cvScalar(255));
      
      
      cv::Point3d tempRay;
      cv::Point2d tempPoint=cv::Point2d(features[i]);
      cam_model.projectPixelTo3dRay(tempPoint,tempRay);
  //    printf("%f x  %f y  %f z\n",tempRay.x,tempRay.y,tempRay.z);
    }
    
  //  featureList[0].print();
    
    //determine error gradient
    
    int min_features=10;
    
  //  printf("ypr %f %f %f\n",yaw,pitch,roll);
    
    cv::Point3d error_sum=calc_error(min_features,0, 0, 0);
    printf("total error is : %f\n",error_sum.x);
    
    for(int i=0;i<featureList.size();i++){
      if(min_features<featureList[i].numFeatures()){
	printf("\n\n\nfeature %d\n",i);
	printf("mean: %f %f %f\n",featureList[i].currentMean.x, featureList[i].currentMean.y, featureList[i].currentMean.z);
	
      }
    }
    
    
//    double error_up= calc_error(min_features,yalpha, 0, 0);
  //  printf("total up yaw error is : %f\n",error_up);
//    double error_down= calc_error(min_features,-yalpha, 0, 0);
  //  printf("total down yaw error is : %f\n",error_down);
  /*  
     
    double yaw_change=0;
    if(error_up<error_sum && error_up<error_down){
      yaw_change=yalpha;
    }else if(error_down<error_sum && error_down<error_up){
      yaw_change=-yalpha;
    }else if(error_down!=error_sum&&error_sum!=error_up){
      yalpha/=2;
    }
    
    error_up=   calc_error(min_features,0,palpha, 0);
   // printf("total up pitch error is : %f\n",error_up);
    error_down=   calc_error(min_features,0,-palpha, 0);
   // printf("total down pitch error is : %f\n",error_down);
    
    double pitch_change=0;
    if(error_up<error_sum && error_up<error_down){
      pitch_change=palpha;
    }else if(error_down<error_sum && error_down<error_up){
      pitch_change=-palpha;
    }else if(error_down!=error_sum&&error_sum!=error_up){
      //palpha/=2;
    }
    
    error_up=  calc_error(min_features,0,0,ralpha);
   // printf("total up roll error is : %f\n",error_up);
    
    error_down=   calc_error(min_features,0,0,-ralpha);
   // printf("total down roll error is : %f\n",error_down);
    
    double roll_change=0;
    if(error_up<error_sum && error_up<error_down){
      roll_change=ralpha;
    }else if(error_down<error_sum && error_down<error_up){
      roll_change=-ralpha;
    }else if(error_down!=error_sum&&error_sum!=error_up){
      ralpha/=2;
    }
    
  //  yaw+=yaw_change;
  
  //  pitch+=pitch_change;
 
  
  //   roll+=roll_change;
    */
    
    try{
      sensor_msgs::Image output_image_cvim =*bridge.cvToImgMsg(output_image, "mono8");
      output_image_cvim.header.stamp=msg->header.stamp;
      analyzed_pub_.publish(output_image_cvim);
    }
    catch (sensor_msgs::CvBridgeException& e){
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
      return;
    }
    // printf("displaying image\n");
  }else{
    // printf("null image_rect\n");
  }
}




int main(int argc, char *argv[]){
  
  ros::init(argc, argv, "vision_analyzer");
  FeatureTracker analyzer;
  cvStartWindowThread();
  /*  cvNamedWindow("rectified image");
  cvNamedWindow("calib image");
  cvNamedWindow("threshold");
  cvNamedWindow("edges");*/
  ros::spin();
  /*	cvDestroyWindow("rectified image");
  cvDestroyWindow("calib image");
  cvDestroyWindow("threshold");
  cvDestroyWindow("edges");*/
  return 0;
}

