#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <string>

class FeatureTracker {
	public:
		FeatureTracker();
		~FeatureTracker();
	private:
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		
		IplImage * image_rect;
		
		IplImage * output_image;
		
		
		IplImage * eigImage;
		IplImage * tempImage;
		int num_features;
		CvPoint2D32f * features;
		double quality_level;
		double min_distance;
		int block_size;
		
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;  
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
		image_transport::Publisher analyzed_pub_;
};

FeatureTracker::FeatureTracker() : it_(nh_), priv_nh_("~"){
	priv_nh_.param("num_features",num_features, 15);
	priv_nh_.param("min_distance", min_distance, 10.0);
	priv_nh_.param("quality_level", quality_level, 0.1);
	priv_nh_.param("block_size", block_size, 7);
  
	image_rect=NULL;
	
	output_image=NULL;
	
	eigImage=NULL;
	tempImage=NULL;
	
	features=new CvPoint2D32f[num_features];
	
	image_subscriber= it_.subscribe("image_rect_color", 1, &FeatureTracker::image_callback, this);
	
	analyzed_pub_= it_.advertise("feature_image", 1);
}

FeatureTracker::~FeatureTracker(){
  delete(features);
}

void FeatureTracker::image_callback(const sensor_msgs::ImageConstPtr& msg) {
 // printf("callback called\n");
  try
	{
	// if you want to work with color images, change from mono8 to bgr8
	  if(image_rect==NULL){
		  image_rect = cvCloneImage(bridge.imgMsgToCv(msg, "mono8"));
		//  printf("cloned image\n");
		}
		else{
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
    int feature_count=num_features;
    
    
    //check if there were features from the last image to keep tracking
    
    //if there were call cvCalcOpticalFlowPyrLK();
    
    //find new features and subpixel them
    
    //find all good features
    cvGoodFeaturesToTrack(image_rect, eigImage, tempImage, features, &feature_count, quality_level, min_distance, NULL, block_size);
    
    //subpixel good features
    /*
    need pose data for each picture, need to publish a camera pose
      geometry_msgs::PoseStamped basePose;
    geometry_msgs::PoseStamped mapPose;
    basePose.pose.orientation.w=1.0;
    ros::Duration timeout(3);
    basePose.header.frame_id="/base_link";
    mapPose.header.frame_id="/map";
    try {
      tf_listener_.waitForTransform("/camera_1_link", "/map", acquisition_time, timeout);
       
      tf_listener_.transformPose("/map", acquisition_time,basePose,"/camera_1_link",mapPose);
	    
	    printf("pose #%d %f %f %f\n",pic_number,mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation));
	    
	    */
    
    //draw dots on image where features are
    for(int i=0;i<feature_count;i++){
      CvPoint center=cvPoint((int)features[i].x,(int)features[i].y);;
      cvCircle(output_image,center,10,cvScalar(150),2);
    }
    
    
    
    try{
      sensor_msgs::Image output_image_cvim =*bridge.cvToImgMsg(output_image, "mono8");
      output_image_cvim.header.stamp=msg->header.stamp;
      analyzed_pub_.publish(output_image_cvim);
	  }
	  catch (sensor_msgs::CvBridgeException& e)
	  {
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

