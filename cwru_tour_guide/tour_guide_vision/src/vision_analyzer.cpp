#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <string>

class VisionAnalyzer {
	public:
		VisionAnalyzer();
	private:
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		
    void warp();
    
    std::string H_path;
    
		IplImage * image_rect;
		
		IplImage * output_image;

    IplImage* calibrated;
    
    CvMat *H;
		double threshold;

	  int32_t output_image_width;
	  int32_t output_image_height;
	  
		double canny_threshold1;
		double canny_threshold2;
		int canny_aperture_size;

    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;  
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
		image_transport::Publisher calib_pub_;
		image_transport::Publisher analyzed_pub_;

};

VisionAnalyzer::VisionAnalyzer() : it_(nh_), priv_nh_("~"){
  
	priv_nh_.param("H_path", H_path,std::string("/tmp/H.xml"));
	priv_nh_.param("threshold",threshold, 255*.9);
	priv_nh_.param("canny_threshold1",canny_threshold1, 10.);
	priv_nh_.param("canny_threshold2",canny_threshold2, 50.);
	priv_nh_.param("canny_aperture_size",canny_aperture_size, 3);
	priv_nh_.param("output_image_width", output_image_width, 600);
	priv_nh_.param("output_image_height", output_image_height, 600);

	image_rect=NULL;
	
	calibrated=cvCreateImage( cvSize(output_image_width,output_image_height), 8, 1 );
	
	output_image =cvCloneImage(calibrated);
	
	try{
	  H =  (CvMat*)cvLoad(H_path.c_str());
	}
	catch(...){
	  printf("H failed to load\n"); 
	  H=NULL;
	}
	if(H==NULL){
	  printf("H did not load %s\n",H_path.c_str());
	}
	
	image_subscriber= it_.subscribe("image_rect_color", 1, &VisionAnalyzer::image_callback, this);
	
	analyzed_pub_= it_.advertise("thresholded_image", 1);
	calib_pub_= it_.advertise("calibrated_image", 1);
}

void VisionAnalyzer::warp(){
  if(H!=NULL&&image_rect!=NULL){
    cvWarpPerspective(
    image_rect,
    calibrated,
    H,
    CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
    );
  }
}

void VisionAnalyzer::image_callback(const sensor_msgs::ImageConstPtr& msg) {
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
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
		return;
	}
	
	if(image_rect!=NULL) {
 //   cvShowImage("rectified image",image_rect);
    warp();
 //   cvShowImage("calib image",calibrated);
    
    cvThreshold(calibrated,output_image,threshold,255, CV_THRESH_BINARY);
    
 //   cvShowImage("threshold",output_image);
    
  //  cvCanny(output_image,output_image,canny_threshold1,canny_threshold2,canny_aperture_size);
    
    
   // cvShowImage("edges",output_image);
    try{
      sensor_msgs::Image output_image_cvim =*bridge.cvToImgMsg(output_image, "mono8");
      output_image_cvim.header.stamp=msg->header.stamp;
      analyzed_pub_.publish(output_image_cvim);
      
      sensor_msgs::Image calibrated_image_cvim =*bridge.cvToImgMsg(calibrated, "mono8");
      calibrated_image_cvim.header.stamp=msg->header.stamp;
      calib_pub_.publish(calibrated_image_cvim);
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
	VisionAnalyzer analyzer;
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

