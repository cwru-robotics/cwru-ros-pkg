#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
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
		void image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo);

    std::string H_path;

    bool project;
		int32_t board_width;
		int32_t board_height;
		int32_t v_board_width;
		int32_t v_board_height;
		double m_per_output_pixel;
		double grid_width;
		double grid_height;
		double m_in_front_of_bot;
		double m_offset_lr_of_bot;
	  int32_t output_image_width;
	  int32_t output_image_height;
	  
	  double dist_to_project_from_board_z;
	  double dist_of_real_board;
	  double dist_to_virtual_board;
		
		IplImage * image_rect;
		
		IplImage * image_corners;

    IplImage* bw_image;
    
    IplImage* calibrated;
    
    CvMat *H;
    
    CvMat *cam;
    
    CvMat *k;
    
    CvMat *rot;
    CvMat *trans;
		
		CvMat *R;
		
		CvMat *point1;
		CvMat *point2;
		CvMat *real_2_virtual_board;
		CvMat *upVector;
		
		CvMat *x_board;
		CvMat *y_board;
		
    CvMat *objPtsMat;
    CvMat *imgPtsMat;
    
    bool calibrated_succesfully;
    bool flip;
  
    sensor_msgs::CvBridge bridge;
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;  
		image_transport::ImageTransport it_;
		image_transport::CameraSubscriber image_subscriber;
};

BirdsEyeCalibrator::BirdsEyeCalibrator() : it_(nh_), priv_nh_("~"){
  
	priv_nh_.param("board_height", board_height, 6);
	priv_nh_.param("board_width", board_width, 7);
	priv_nh_.param("grid_width", grid_width, 0.02858);
	priv_nh_.param("grid_height", grid_height, 0.02858);
	priv_nh_.param("m_in_front_of_bot", m_in_front_of_bot, 0.20);
	priv_nh_.param("m_offset_lr_of_bot", m_offset_lr_of_bot, 0.0);
	priv_nh_.param("m_per_output_pixel", m_per_output_pixel, 0.001);
	priv_nh_.param("output_image_width", output_image_width, 600);
	priv_nh_.param("output_image_height", output_image_height, 600);
	priv_nh_.param("flip", flip,false);
	priv_nh_.param("m_dist_to_projection_from_board_z", dist_to_project_from_board_z, 1.0);
	priv_nh_.param("project", project,false);

	priv_nh_.param("H_path", H_path,std::string("/tmp/H.xml"));
	
	image_rect=NULL;
	calibrated=NULL;
	bw_image=NULL;
	image_corners=NULL;
	calibrated_succesfully=false;
  
	
	H = cvCreateMat( 3, 3, CV_32F);
	
	cam = cvCreateMat( 3, 3, CV_32F);
	k = cvCreateMat( 1, 4, CV_32F);
	
	objPtsMat=cvCreateMat( 4, 3, CV_32F);
	
	imgPtsMat=cvCreateMat( 4, 2, CV_32F);
	
	rot=cvCreateMat(3, 1, CV_32F);
	trans=cvCreateMat(3, 1, CV_32F);
	
	R=cvCreateMat(3, 3, CV_32F);
	
	point1=cvCreateMat(3, 1, CV_32F);
	point2=cvCreateMat(3, 1, CV_32F);
	real_2_virtual_board=cvCreateMat(3, 1, CV_32F);
	x_board=cvCreateMat(3, 1, CV_32F);
	y_board=cvCreateMat(3, 1, CV_32F);
	
	upVector=cvCreateMat(3, 1, CV_32F);
	
	image_subscriber= it_.subscribeCamera("image_rect_color", 1, &BirdsEyeCalibrator::image_callback, this);
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
			//cvSaveImage("/tmp/output.jpg",image_rect);
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
    
    double xoffset=output_image_width/2.;
    double yoffset=output_image_height/2.;
   
  //  double xoffset=output_image_width/2.;
  //  double yoffset=output_image_height/2.;
    
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
			printf("looking for %d x %d corners\
      calibrated_succesfully=false\n",board_width,board_height);
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
    
    CvPoint2D32f realObjPts[4];
    realObjPts[0].x = 0;          realObjPts[0].y = 0;
    realObjPts[1].x = grid_width*(board_width-1);  realObjPts[1].y = 0;
    realObjPts[2].x = 0;          realObjPts[2].y = grid_height*(board_height-1);
    realObjPts[3].x = grid_width*(board_width-1);  realObjPts[3].y = grid_height*(board_height-1);
    
    imgPts[0]   = corners[0];
    imgPts[1]   = corners[board_width-1];
    imgPts[2]   = corners[(board_height-1)*board_width];
    imgPts[3]   = corners[(board_height-1)*board_width + board_width-1];
    
    for(int i=0;i<4;i++){
      CV_MAT_ELEM(*objPtsMat,float,i,0)=realObjPts[i].x;
      CV_MAT_ELEM(*objPtsMat,float,i,1)=realObjPts[i].y;
      CV_MAT_ELEM(*objPtsMat,float,i,2)=0;
    }
    
    for(int i=0;i<4;i++){
      CV_MAT_ELEM(*imgPtsMat,float,i,0)=imgPts[i].x;
      CV_MAT_ELEM(*imgPtsMat,float,i,1)=imgPts[i].y;
    }
    
    cvFindExtrinsicCameraParams2(objPtsMat,imgPtsMat,cam,NULL,rot,trans,0);
    
    printf("rot mat \n %f %f %f\n\n",CV_MAT_ELEM(*rot,float,0,0),CV_MAT_ELEM(*rot,float,1,0),CV_MAT_ELEM(*rot,float,2,0));

    printf("trans mat \n %f %f %f\n\n",CV_MAT_ELEM(*trans,float,0,0),CV_MAT_ELEM(*trans,float,1,0),CV_MAT_ELEM(*trans,float,2,0));
    cvRodrigues2(rot,R,NULL);

    //cvCopy(trans,point1,NULL);

    cvSetZero(point2);
    CV_MAT_ELEM(*point2,float,2,0)=1.0;

    cvGEMM(R,point2,1,trans,1,point1,0);

    cvSub(point1,trans,upVector);
 
    double len=cvNorm(upVector,NULL,CV_L2,0);

    printf("raw vector \n %f %f %f \n",CV_MAT_ELEM(*point1,float,0,0),CV_MAT_ELEM(*point1,float,1,0),CV_MAT_ELEM(*point1,float,2,0));

    printf("len is %f\n and \n %f %f %f \n",len,CV_MAT_ELEM(*upVector,float,0,0),CV_MAT_ELEM(*upVector,float,1,0),CV_MAT_ELEM(*upVector,float,2,0));
    
    dist_of_real_board= cvNorm(trans,NULL,CV_L2,0);

//find the vector going out of the board plane and away from the camera in the camera frame
     
    //then project the vector from the camera to 0,0 into the vector going out of the board
    //determine how much that vector must me scaled by to become the length of
    //dist_to_project_from_board_z
    //then scale the original vector from the camera to 0,0 by that scale

    //double=cvDotProduct(a,b)
    //double=cvNorm(a,NULL,CV_L2,0);

    //v = upVector dot trans /len *upVector/len
    // scale=dist_to_project_from_board_z/length(v)
    
    double val=cvDotProduct(upVector,trans);
    double scale=dist_to_project_from_board_z/val;
    
    //this is the vector from 0,0 on the real board, to 0,0 on the virtual board
    cvScale(trans,real_2_virtual_board,scale);
    
    dist_to_virtual_board=dist_of_real_board+ cvNorm(real_2_virtual_board,NULL,CV_L2,0);
    
    printf("dist to real %f dist to virt %f\n",dist_of_real_board,dist_to_virtual_board);
    
    //use the ratio of the distance to the real board vs the virtual board to change
    //the image points for the calibration

    grid_width=(grid_width/dist_of_real_board)*dist_to_virtual_board;
    grid_height=(grid_height/dist_of_real_board)*dist_to_virtual_board;
    
    //also project the vector from real to virtual in the 0,1 and 1,0 on board vectors
    //to determin what the x y offset is in the robot frame between the boards so we can
    //build a map properly
    
    cvSetZero(point1);
    CV_MAT_ELEM(*point1,float,0,0)=-1.0;
    
    cvGEMM(R,point1,1,trans,1,point2,0);
    
    cvSub(point2,trans,x_board);
    
    cvSetZero(point1);
    CV_MAT_ELEM(*point1,float,1,0)=-1.0;
    
    cvGEMM(R,point1,1,trans,1,point2,0);
    cvSub(point2,trans,y_board);
    
    double x_add=cvDotProduct(real_2_virtual_board,x_board)/cvNorm(x_board,NULL,CV_L2,0);
    double y_add=cvDotProduct(real_2_virtual_board,y_board)/cvNorm(y_board,NULL,CV_L2,0);
    
    printf("xadd %f yadd %f\n",x_add,y_add);
    if(flip){
      imgPts[1]   = corners[0];
      imgPts[0]   = corners[board_width-1];
      imgPts[3]   = corners[(board_height-1)*board_width];
      imgPts[2]   = corners[(board_height-1)*board_width + board_width-1];
      x_add-=grid_width*(board_width-1);
      xoffset+=(x_add+ m_offset_lr_of_bot)/m_per_output_pixel;
    }
    else{
      xoffset-=(x_add+ m_offset_lr_of_bot)/m_per_output_pixel;
    }
    
    printf("x offset width %f\n",x_add+ m_offset_lr_of_bot);
    
    yoffset-=(y_add + m_in_front_of_bot)/m_per_output_pixel;
    
    //Change the object points here
    
    if(project){
      objPts[0].x = 0+xoffset;          objPts[0].y = 0+yoffset;
      objPts[1].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[1].y = 0+yoffset;
      objPts[2].x = 0+xoffset;          objPts[2].y = grid_height/m_per_output_pixel*(board_height-1)+yoffset;
      objPts[3].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[3].y = grid_height/m_per_output_pixel*(board_height-1)+yoffset;
    }
    else{
      xoffset=output_image_width/2.;
      yoffset=output_image_height/2.;
      objPts[0].x = 0+xoffset;          objPts[0].y = 0+yoffset;
      objPts[1].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[1].y = 0+yoffset;
      objPts[2].x = 0+xoffset;          objPts[2].y = grid_height/m_per_output_pixel*(board_height-1)+yoffset;
      objPts[3].x = grid_width/m_per_output_pixel*(board_width-1)+xoffset;  objPts[3].y = grid_height/m_per_output_pixel*(board_height-1)+yoffset;
    }
    
    cvGetPerspectiveTransform( objPts, imgPts, H);
    //dist_of_real_board=CV_MAT_ELEM(*H,float,2,2);
    
    //printf("board %f\n",dist_of_real_board);
    
    //v_board_width= (dist_to_project_from_board_z + dist_of_real_board)/(dist_of_real_board/board_width);
      
    //DRAW THE POINTS in order: B,G,R,YELLOW
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

void BirdsEyeCalibrator::image_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo) {
//  printf("callback called\n");

  CV_MAT_ELEM(*cam,float,0,0)=cinfo->P[0];
  CV_MAT_ELEM(*cam,float,0,1)=cinfo->P[1];
  CV_MAT_ELEM(*cam,float,0,2)=cinfo->P[2];
  CV_MAT_ELEM(*cam,float,1,0)=cinfo->P[4];
  CV_MAT_ELEM(*cam,float,1,1)=cinfo->P[5];
  CV_MAT_ELEM(*cam,float,1,2)=cinfo->P[6];
  CV_MAT_ELEM(*cam,float,2,0)=cinfo->P[8];
  CV_MAT_ELEM(*cam,float,2,1)=cinfo->P[9];
  CV_MAT_ELEM(*cam,float,2,2)=cinfo->P[10];
  
//   printf("cinfok \n %f %f %f\n %f %f %f\n %f %f %f\n\n",cinfo->K[0],cinfo->K[1],cinfo->K[2],cinfo->K[3],cinfo->K[4],cinfo->K[5],cinfo->K[6],cinfo->K[7],cinfo->K[8]);
    
//   printf("cam mat \n %f %f %f\n %f %f %f\n %f %f %f\n\n",CV_MAT_ELEM(*cam,float,0,0),CV_MAT_ELEM(*cam,float,0,1),CV_MAT_ELEM(*cam,float,0,2),CV_MAT_ELEM(*cam,float,1,0),CV_MAT_ELEM(*cam,float,1,1),CV_MAT_ELEM(*cam,float,1,2),CV_MAT_ELEM(*cam,float,2,0),CV_MAT_ELEM(*cam,float,2,1),CV_MAT_ELEM(*cam,float,2,2));
   
//  printf("cinfo \n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n\n",cinfo->P[0],cinfo->P[1],cinfo->P[2],cinfo->P[3],cinfo->P[4],cinfo->P[5],cinfo->P[6],cinfo->P[7],cinfo->P[8],cinfo->P[9],cinfo->P[10],cinfo->P[11]);
   
   // printf("cinfo D \n %f %f %f %f %f\n\n",cinfo->D[0],cinfo->D[1],cinfo->D[2],cinfo->D[3],cinfo->D[4]);
  
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

