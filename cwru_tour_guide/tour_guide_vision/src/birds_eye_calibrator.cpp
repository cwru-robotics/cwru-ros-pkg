#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <string>

class BirdsEyeCalibrator {
	public:
		BirdsEyeCalibrator();
		void mainLoop();
		void calibrate();
		void warp();
	private:
		void image_callback(const sensor_msgs::ImageConstPtr& msg);

    std::string H_path;

		int32_t board_width;
		int32_t board_height;
		double m_per_output_pixel;
		double grid_width;
		double grid_height;
		double m_in_front_of_bot;
		double m_offset_lr_of_bot;
	  int32_t output_image_width;
	  int32_t output_image_height;
		
		IplImage * image_rect;
		
		IplImage * image_corners;

    IplImage* bw_image;
    
    IplImage* calibrated;
    
    CvMat *H;
    
    bool calibrated_succesfully;
  
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
};

BirdsEyeCalibrator::BirdsEyeCalibrator() : it_(nh_){
  
	ros::NodeHandle nh_;
	nh_.param("board_height", board_height, 6);
	nh_.param("board_width", board_width, 8);
	nh_.param("grid_width", grid_width, 0.02858);
	nh_.param("grid_height", grid_height, 0.02858);
	nh_.param("m_in_front_of_bot", m_in_front_of_bot, 0.20);
	nh_.param("m_offset_lr_of_bot", m_offset_lr_of_bot, 0.0);
	nh_.param("m_per_output_pixel", m_per_output_pixel, 0.001);
	nh_.param("output_image_width", output_image_width, 600);
	nh_.param("output_image_height", output_image_height, 600);
	nh_.param("H_path", H_path,std::string("./H.xml"));
	
	image_rect=NULL;
	calibrated=NULL;
	bw_image=NULL;
	image_corners=NULL;
	calibrated_succesfully=false;
  
	
	H = cvCreateMat( 3, 3, CV_32F);
	
	image_subscriber= it_.subscribe("image_rect_color", 1, &BirdsEyeCalibrator::image_callback, this);
}

void BirdsEyeCalibrator::mainLoop(){

  calibrated=cvCreateImage( cvSize(output_image_width, output_image_height), 8, 3 );
  
  //start displaying
  ros::Rate r(20);
  int key=-1;
  
  while(ros::ok()){
    if(image_rect!=NULL) {
      cvShowImage("rectified image",image_rect);
		 // printf("displaying image\n");
    }else{
     // printf("null image_rect\n");
    }
    
    if(calibrated_succesfully){
      warp();
      cvShowImage("calibrated image",calibrated);
    }
    
    
    key=cvWaitKey(10);
    
   // printf("%d\n",key);
    if(32== (255&key)){ 
      //space was hit and calibrate
      calibrate();
    }else if(115== (255&key)){
      //s was hit and save the calibration
      if(calibrated_succesfully){
        cvSave(H_path.c_str(),H); //We can reuse H for the same camera mounting
        printf("saved calibration matrix to %s\n",H_path.c_str());
      }
      else{
        printf("Did not save because there is not a valid calibration in current H matrix\n");
      }
    }else if(113== (255&key) || 27== (255&key) ){
      //esc or q was hit and exit
      ros::shutdown();
    }
    
    ros::spinOnce();
    r.sleep();
  }
}

void BirdsEyeCalibrator::warp(){
  if(calibrated_succesfully){
    cvWarpPerspective(
	  image_rect,
	  calibrated,
	  H,
	  CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
	  );
	}
	else{
	  printf("Did not warp because there is not a valid calibration in current H matrix\n");
	}
}

void BirdsEyeCalibrator::calibrate(){
  calibrated_succesfully=false;
  if(image_rect!=NULL){
    if(bw_image==NULL){
      bw_image=cvCreateImage(  cvGetSize(image_rect), 8, 1 );
    }
    if(image_corners==NULL){
      image_corners=cvCloneImage(image_rect);
    }
    cvCopy(image_rect,image_corners);
    cvCvtColor(image_rect, bw_image, CV_BGR2GRAY );
    
    int board_n = board_width * board_height;
    CvSize board_sz = cvSize( board_width, board_height);
    
    CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
    
    int corner_count = 0;
    
    double xoffset=output_image_width/2. - m_offset_lr_of_bot/m_per_output_pixel;
    double yoffset=output_image_height/2.- m_in_front_of_bot/m_per_output_pixel;
    
    int found = cvFindChessboardCorners(
	  bw_image,
	  board_sz,
	  corners,
	  &corner_count,
	  CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
    );
    
    if(!found){
	    printf("Couldn't aquire chessboard on image, "
	      "only found %d of %d corners\n",corner_count,board_n
	    );
      calibrated_succesfully=false;
	    return;
    }
    
    cvFindCornerSubPix(
	    bw_image,
	    corners,
	    corner_count,
	    cvSize(11,11),
	    cvSize(-1,-1),
	    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 )
      );
    
    CvPoint2D32f objPts[4], imgPts[4];
    objPts[0].x = 0+xoffset;          objPts[0].y = 0+yoffset;
    objPts[1].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[1].y = 0+yoffset;
    objPts[2].x = 0+xoffset;          objPts[2].y = grid_width/m_per_output_pixel*(board_height-1)+yoffset;
    objPts[3].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[3].y = grid_width/m_per_output_pixel*(board_height-1)+yoffset;
    
    imgPts[0]   = corners[0];
    imgPts[1]   = corners[board_width-1];
    imgPts[2]   = corners[(board_height-1)*board_width];
    imgPts[3]   = corners[(board_height-1)*board_width + board_width-1];
    
    cvGetPerspectiveTransform( objPts, imgPts, H);
    
      
      
    // DRAW THE POINTS in order: B,G,R,YELLOW
    //
    cvCircle( image_corners, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),   3);
    cvCircle( image_corners, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),   3);
    cvCircle( image_corners, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),   3);
    cvCircle( image_corners, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0), 3);

    // DRAW THE FOUND CHESSBOARD
    //
    cvDrawChessboardCorners(
	  image_corners,
	  board_sz,
	  corners,
	  corner_count,
	  found
    );
    
    cvShowImage( "corners found", image_corners );
  
    printf("found corners\n");
    calibrated_succesfully=true;
  }
}

void BirdsEyeCalibrator::image_callback(const sensor_msgs::ImageConstPtr& msg) {
//  printf("callback called\n");
  try
	{
	  if(image_rect==NULL){
		  image_rect = cvCloneImage(bridge.imgMsgToCv(msg, "bgr8"));
		//  printf("cloned image\n");
		}
		else{
		  cvCopy(bridge.imgMsgToCv(msg, "bgr8"),image_rect);
		//  printf("copied image\n");
		}
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char *argv[]){
  
 	ros::init(argc, argv, "birds_eye_calibrator");
	BirdsEyeCalibrator calibrator;
  cvNamedWindow("rectified image");
  cvNamedWindow("corners found");
  cvNamedWindow("calibrated image");
	//cvStartWindowThread(); //comment this out since we are using keyboard inputs as commands
  calibrator.mainLoop();
	cvDestroyWindow("rectified image");
  cvDestroyWindow("corners found");
	cvDestroyWindow("calibrated image");
	
  return 0;
}

