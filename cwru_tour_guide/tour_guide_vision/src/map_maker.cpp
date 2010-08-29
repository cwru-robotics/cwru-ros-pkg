#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <string>

class MapMaker {
	public:
		MapMaker();
	private:
		void image_callback(const sensor_msgs::ImageConstPtr& msg);
    
		IplImage * input_image;
		IplImage * rotationImage;
		
		IplImage * map;
		
	  int32_t map_image_width;
	  int32_t map_image_height;
	  
	  int32_t map_zero_x;
	  int32_t map_zero_y;
	  int32_t init_zero_x;
	  int32_t init_zero_y;
	  double map_meters_per_pixel;

    int pic_number;
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;  
		tf::TransformListener tf_listener_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
};

MapMaker::MapMaker() : it_(nh_), priv_nh_("~"){
  printf("constructing\n");
  priv_nh_.param("map_image_width", map_image_width, 1000);
	priv_nh_.param("map_image_height", map_image_height, 1000);
	priv_nh_.param("map_zero_x", map_zero_x, 500);
	priv_nh_.param("map_zero_y", map_zero_y, 500);
	priv_nh_.param("map_meters_per_pixel", map_meters_per_pixel, 0.005);
	
  input_image=NULL;
  rotationImage=NULL;
	map=cvCreateImage( cvSize(map_image_width,map_image_height), 8, 1 );
	image_subscriber= it_.subscribe("map_input_image", 1, &MapMaker::image_callback, this);
	
	init_zero_x=0;
	init_zero_y=0;
  pic_number=0;

  printf("object made\n");
}

void MapMaker::image_callback(const sensor_msgs::ImageConstPtr& msg) {
//  printf("callback called\n");
  try
	{
	
	// if you want to work with color images, change from mono8 to bgr8
	  if(input_image==NULL){
		  input_image = cvCloneImage(bridge.imgMsgToCv(msg, "mono8"));
		  rotationImage=cvCloneImage(input_image);
		 // printf("cloned image\n");
		}
		else{
		  cvCopy(bridge.imgMsgToCv(msg, "mono8"),input_image);
		 // printf("copied image\n");
		}
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
		return;
	}
	
	if(input_image!=NULL) {
    //get tf transform here and put in map
    ros::Time acquisition_time = msg->header.stamp;
    geometry_msgs::PoseStamped basePose;
    geometry_msgs::PoseStamped mapPose;
    basePose.pose.orientation.w=1.0;
    ros::Duration timeout(3);
    basePose.header.frame_id="/base_link";
    mapPose.header.frame_id="/map";
    try {
      tf_listener_.waitForTransform("/base_link", "/map", acquisition_time, timeout);
       
      tf_listener_.transformPose("/map", acquisition_time,basePose,"/base_link",mapPose);
	    
	    printf("pose #%d %f %f %f\n",pic_number,mapPose.pose.position.x, mapPose.pose.position.y, tf::getYaw(mapPose.pose.orientation));
	    
	    
	    /*
	    char buffer [50];
	    sprintf (buffer, "/tmp/test%02d.jpg", pic_number);
			if(!cvSaveImage(buffer,input_image,0)) printf("Could not save: %s\n",buffer);
			else printf("picture taken!!!\n");
	    pic_number++;
	    */
	    
	    cv::Point_<double> center;
      center.x=input_image->width/2;
      center.y=input_image->height/2;
      double tranlation_arr[2][3];
      CvMat translation;
      cvInitMatHeader(&translation,2,3,CV_64F,tranlation_arr);
      
      cvSetZero(&translation);
      cv2DRotationMatrix(center, (tf::getYaw(mapPose.pose.orientation)*180/3.14159) -90,1.0,&translation);
      cvSetZero(rotationImage);
      cvWarpAffine(input_image,rotationImage,&translation,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
      
      
      CvRect roi;
      roi.width=rotationImage->width;
      roi.height=rotationImage->height;
      
      if(init_zero_x==0){
        init_zero_x=(int)(mapPose.pose.position.x*(1.0/map_meters_per_pixel));
        init_zero_y=(int)(mapPose.pose.position.y*(-1.0/map_meters_per_pixel));
      }
      
      roi.x=(int)(mapPose.pose.position.x*(1.0/map_meters_per_pixel))-init_zero_x+map_zero_x-roi.width/2;
      roi.y=(int)(mapPose.pose.position.y*(-1.0/map_meters_per_pixel))-init_zero_y+map_zero_y-roi.height/2;
      
      printf("x %d, y %d, rot %f\n",roi.x,roi.y, (tf::getYaw(mapPose.pose.orientation)*180/3.14159) -90);
      
      cvSetImageROI(map,roi);
      
      cvMax(map,rotationImage,map);
      
      cvResetImageROI(map);
	    cvShowImage("map image",map);	    
    }
    catch (tf::TransformException& ex) {
      ROS_WARN("[map_maker] TF exception:\n%s", ex.what());
      printf("[map_maker] TF exception:\n%s", ex.what());
      return;
    }
    catch(...){
      printf("opencv shit itself cause our roi is bad\n");
    }
  }
}


int main(int argc, char *argv[]){
  
 	ros::init(argc, argv, "map_maker");
	MapMaker mapmaker;
	cvStartWindowThread();
  cvNamedWindow("map image");
  ros::spin();
	cvDestroyWindow("map image");

  return 0;
}

