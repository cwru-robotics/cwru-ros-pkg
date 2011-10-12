// source: http://www.codingbeta.com/?p=24
// Headers
//#include "stdafx.h" // commenting out this line because i think it's just a visual studio thing
// Include OpenNI
#include "openni/XnCppWrapper.h"
// Include NITE
#include "nite/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


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
	
	// We wish to print the middle pixel's depth, get the index
	// of the middle pixel.
	XnUInt32 nMiddleIndex = XN_QQVGA_X_RES * XN_QVGA_Y_RES/ 2 + 
		XN_QVGA_X_RES/2;										

	// define the range of pixels to search
	XnUInt32 searchWidth=20;
	XnUInt32 searchHeight=searchWidth;
	XnUInt32 xStart = XN_QVGA_X_RES/2-searchWidth/2;	
	XnUInt32 yStart = XN_QVGA_Y_RES/2-searchHeight/2;	
	XnUInt32 tempDepth;
	XnUInt32 distanceThreshold=1000; // this is in mm

	bool isObstacle=false;

	// Main loop
	while (true)
	{
		// Wait for new data to be available
		nRetVal = context.WaitOneUpdateAll(depth);
		CHECK_RC(nRetVal, "Updating depth");
		// Get the new depth map
		const XnDepthPixel* pDepthMap = depth.GetDepthMap();
		// Print out the value of the middle pixel
		//printf("Middle pixel is %u millimeters away\n", pDepthMap[nMiddleIndex]);


		//printf("got there line 77\n");
		isObstacle=false;

		for(int i=xStart; i<xStart+searchWidth; i++){
			for(int j=yStart; j<yStart+searchHeight; j++){
				tempDepth=pDepthMap[XN_QQVGA_X_RES * j + i];
				//printf("got there line 83\n");
				if(tempDepth<distanceThreshold && tempDepth>0){
					isObstacle=true;
				}
			}
		}
		if(isObstacle){
			printf("oh noes there is an obstacle!!!!1\n");
		}
		else{
			printf("you're okay man\n");
		}
	}

	// Clean-up
	context.Shutdown();
	return 0;
}

