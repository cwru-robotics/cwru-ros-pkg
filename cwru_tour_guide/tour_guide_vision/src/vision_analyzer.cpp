#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
    
// I need some way of mapping the H matricies with the images, via params
    std::string H_path;

		IplImage * image_rect;
		
		IplImage * output_image;

    IplImage* calibrated;
    
    CvMat *H;
  
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
};

VisionAnalyzer::VisionAnalyzer() : it_(nh_){
  
	ros::NodeHandle nh_;
//	nh_.param("H_path", H_path,std::string("./H.xml"));
	
	image_rect=NULL;
	calibrated=NULL;
	output_image =NULL;
	
	H = cvCreateMat( 3, 3, CV_32F);
	
	image_subscriber= it_.subscribe("image_rect_color", 1, &VisionAnalyzer::image_callback, this);
}

void VisionAnalyzer::warp(){
  cvWarpPerspective(
  image_rect,
  calibrated,
  H,
  CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
  );
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
		//  printf("copied image\n");
		}
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		return;
	}
	
	if(image_rect!=NULL) {
    cvShowImage("rectified image",image_rect);
    
    if(output_image==NULL){
      output_image=cvCloneImage(image_rect);
    }
    cvThreshold(image_rect,output_image,254,255, CV_THRESH_BINARY);
    
    cvShowImage("threshold",output_image);
    
    cvCanny(output_image,output_image,10,50,3);
    
    
    cvShowImage("edges",output_image);
    

	 // printf("displaying image\n");
  }else{
   // printf("null image_rect\n");
  }
  
  
	
}


int main(int argc, char *argv[]){
  
 	ros::init(argc, argv, "vision_analyzer");
	VisionAnalyzer calibrator;
	cvStartWindowThread();
  cvNamedWindow("rectified image");
  cvNamedWindow("threshold");
  cvNamedWindow("edges");
	//cvStartWindowThread(); //comment this out since we are using keyboard inputs as commands
  ros::spin();
	cvDestroyWindow("rectified image");
  cvDestroyWindow("threshold");
  cvDestroyWindow("edges");
  return 0;
}

