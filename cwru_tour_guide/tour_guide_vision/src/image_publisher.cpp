#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/CvBridge.h>
#include <string>

class VisionAnalyzer {
	public:
		VisionAnalyzer();
		void publish();
	private:
    
    std::string image_path;
    std::string calib_path;
    
    
		IplImage * output_image;
	//	IplImage * bw_image;
		int publish_rate;
    
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;  
		image_transport::ImageTransport it_;
		image_transport::CameraPublisher output_pub_;
		
	  sensor_msgs::CameraInfo cam_info_;
	  CameraInfoManager cinfo_;

};

VisionAnalyzer::VisionAnalyzer() : nh_("camera_spoof"),it_(nh_),cinfo_(nh_), priv_nh_("~"){
  
	priv_nh_.param("image_path", image_path,std::string("/tmp/image.jpg"));
	priv_nh_.param("publish_rate", publish_rate,1);
	priv_nh_.param("camera_info_url",calib_path ,std::string(""));
	
	output_image=NULL;
	try{
	  output_image =  cvLoadImage(image_path.c_str());
	//  bw_image =  cvCreateImage( cvGetSize(output_image),8, 1 );
  //  cvCvtColor(output_image, bw_image, CV_BGR2GRAY );
	}
	catch(...){
	  printf("image failed to load\n"); 
	  output_image=NULL;
	}
	
	
	try{
	  if(!calib_path.empty()&&cinfo_.validateURL(calib_path)){
	    cinfo_.loadCameraInfo(calib_path);
	    
		  cam_info_ = cinfo_.getCameraInfo();
	  }
	}catch(...){
	  printf("camera calibration failed to load\n"); 
	}
	
 // cvShowImage("output image",output_image);
	output_pub_= it_.advertiseCamera("image_raw", 1);
}

void VisionAnalyzer::publish(){

	ros::Rate loop_rate(publish_rate);
	
  while(ros::ok()){
    try{
      sensor_msgs::Image output_image_cvim =*bridge.cvToImgMsg(output_image, "bgr8");
		  output_pub_.publish(output_image_cvim, cam_info_);
    }
    catch (sensor_msgs::CvBridgeException& e)
    {
      ROS_ERROR("Could not convert to 'bgr8'.");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char *argv[]){
  
 	ros::init(argc, argv, "vision_analyzer");
	VisionAnalyzer analyzer;
	cvStartWindowThread();
 // cvNamedWindow("output image");
  
  analyzer.publish();
	//cvDestroyWindow("output image");
  return 0;
}

