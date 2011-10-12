// this is dumb: export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mobilerobots/kinect/proximity_detection

// Include OpenNI
#include "openni/XnCppWrapper.h"
// Include NITE
#include "nite/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <math.h>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

// to do the ros msg i have this source http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors
#include "sensor_msgs/LaserScan.h"

using namespace std;


// This macro checks the return code that all OpenNI calls make
// and throws an error if the return code is an error. Use this
// after all calls to OpenNI objects. Great for debugging.
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

#define _TCHAR char
#define _tmain main

double* xYToRTheta(double x, double y);
double* rotate(double x, double y, double z, double theta);

int _tmain(int argc, _TCHAR* argv[])
{
	//
	// Variables

	// Keep track of the return code of all OpenNI calls
	XnStatus nRetVal = XN_STATUS_OK;
	// context is the object that holds most of the things related to OpenNI
	xn::Context context;
	// The DepthGenerator generates a depth map that we can then use to do 
	// cool stuff with. Other interesting generators are gesture generators
	// and hand generators.
	xn::DepthGenerator depth;

	//
	// Initialization
	
	// Initialize context object
	nRetVal = context.Init();
	CHECK_RC(nRetVal, "Initialize context");
	// Create the depth object
	nRetVal = depth.Create(context);
	CHECK_RC(nRetVal, "Create Depth");
	
	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "Start Generating All Data");

	// to publish:
	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("kinect_scan", 50);
//	ros::Publisher scan_pub = n.advertise<LaserScan>("scan", 50);

  double laser_frequency = 40;
  //double ranges[2*angleMax];
  //double intensities[2*angleMax];

  //int count = 0;
  ros::Rate r(5.0);


	
				

	int angleMax=90;
	int* distances=new int[2*angleMax];
	int unknownDist=-1; // so, this is in mm, yes?

	double* rotatedResult;
	double* tempRTheta;
	double floorThreshold=-950; //-950; // magical floor threshold: -950 yay!

	int angleIndex;
	int angleIndex2;
	int tiltAngle=-13; //-20; //60; //45; //15; // in degrees
	// magical angle: -13 yay!

	bool notWritten=true;

	int total_iterations=10;
	int count_iterations=0;

	// Main loop
	while (/*count_iterations++<total_iterations && */ n.ok())
	{
		// Wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		CHECK_RC(nRetVal, "Updating depth");
		// Get the new depth map
		const XnDepthPixel* pDepthMap = depth.GetDepthMap();

		int real_x_dim=640;
		int real_y_dim=480;

		//XnVector3D realWorld[XN_VGA_Y_RES*XN_VGA_X_RES];
		XnPoint3D realWorld[real_x_dim*real_y_dim];

		// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
		XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];

		double Z;
		int count3=0;
		for (int y=0; y<XN_VGA_Y_RES; y++){
			for(int x=0;x<XN_VGA_X_RES;x++){

			        pointList[y * XN_VGA_X_RES + x ].X =x;
			        pointList[y * XN_VGA_X_RES + x ].Y =y;
			        pointList[y * XN_VGA_X_RES + x ].Z = (short) pDepthMap[y *XN_VGA_X_RES + x];

				count3++;
			}
		} 


		depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 
		// hey guys, x is horizontal, y is vertical, z is depth.

		//printf("length = %f\n",(double)(sizeof(realWorld)/sizeof(XnPoint3D)));

		for(int i=0; i<2*angleMax; i++){
			distances[i]=unknownDist;
		}


		int x_res=XN_VGA_X_RES;
		int y_res=XN_VGA_Y_RES;

		for(int j=0; j<y_res; j++){ // as opposed to XN_QQVGA_X_RES
			for(int i=0; i<x_res; i++){
				rotatedResult=rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle);

				if(rotatedResult[1]>floorThreshold){
					//printf("got ther eline 123\n");
					tempRTheta=xYToRTheta(rotatedResult[2],realWorld[j*x_res + i].X);
					angleIndex=angleMax-(int)tempRTheta[1];

					if(distances[angleIndex]<0 || distances[angleIndex]>tempRTheta[0]){
						//printf("angle = %d angleIndex = %d\n",(int)(tempRTheta[1]),angleIndex);
						if(angleIndex<=90){
							angleIndex2=90-angleIndex;
						}
						else{
							angleIndex2=270-angleIndex;
						}
						distances[angleIndex2]=(int)(tempRTheta[0]);
					}
				}
			}
		}
/*
		// write to file
		ofstream myfile;
		myfile.open ("lidar_result.txt");
		if(!myfile){
			printf("oh my goodness, the file didn't open\n");
		}
		for(int i=0;i<2*angleMax; i++){
			//myfile << "Writing this to a file.\n";
			myfile << distances[i] << endl;
			//printf("wrote to file, i = %d\n",i);
		}

		myfile.close();
*/
		//count_iterations++;
		//printf("count_iterations = %d\n",count_iterations);


    
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / (2*angleMax);
    scan.time_increment = (1 / laser_frequency) / (2*angleMax);
    scan.range_min = -1; //0.0;
    scan.range_max = 100.0;

    scan.set_ranges_size(2*angleMax);
    //scan.set_intensities_size(2*angleMax);
    for(unsigned int i = 0; i < 2*angleMax; ++i){
      scan.ranges[i] = distances[i];
      //scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
   // ++count;
    r.sleep();
}



	// Clean-up
	context.Shutdown();
	return 0;
}


double* xYToRTheta(double x, double y){
	double* result= new double[2];
	result[0]=sqrt(x*x+y*y);
	if(y!=0){
		result[1]=atan(x/y)*180/3.14159;
	}
	else{
		result[1]=0;
	}
	return result;
}
double* rotate(double x, double y, double z, double theta){
	// so for this case, it needs to be (x, depth, height)
	double* result=new double[3];
	double thetaRad=theta*3.14159/180;
	result[0]=x;
	result[1]=cos(thetaRad)*y - sin(thetaRad)*z;
	result[2]=sin(thetaRad)*y + cos(thetaRad)*z;
	return result;
}
