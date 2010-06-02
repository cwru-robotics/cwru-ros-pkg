#include "jaus/mobility/sensors/globalposesensor.h"
#include "jaus/mobility/sensors/localposesensor.h"
#include "jaus/mobility/sensors/velocitystatesensor.h"
#include "jaus/core/component.h"
#include <cxutils/keyboard.h>
#include <cxutils/math/cxmath.h>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>

//ROS includes
#include <ros/ros.h>
#include <harlie_base/Pose.h>

#ifdef WIN32
#ifndef WIN64
//#include <vld.h>
#endif
#endif

JAUS::UShort gSubsystemID   = 3000;   // CHANGE HERE for Subsystem ID!!!
JAUS::Byte gNodeID          = 1;
JAUS::Byte gComponentID     = 1;      

JAUS::GlobalPoseSensor* globalPoseSensor;
JAUS::LocalPoseSensor* localPoseSensor;
JAUS::VelocityStateSensor* velocityStateSensor;

double offset_lat;
double offset_long;
double lat_conversion_to_m;
double long_conversion_to_m;
double glolat;
double glolong;
double glotheta;
double gomega;
double govel;

void callback(const harlie_base::PoseConstPtr& pose) {
	//cRIO pose... we use this because it is already in the JAUS frame and doesn't need coordinate system munging to function
//	JAUS::GlobalPose globalPose;
//	JAUS::VelocityState velocityState;
	
//	globalPose.SetLatitude((pose->x/lat_conversion_to_m) + offset_lat);
//	globalPose.SetLongitude((pose->y/long_conversion_to_m) + offset_long);
//	globalPose.SetYaw(pose->theta);
//	globalPose.SetTimeStamp(JAUS::Time::GetUtcTime());
	
	glolat = (pose->x/lat_conversion_to_m) + offset_lat;
	glolong = (pose->y/long_conversion_to_m) + offset_long;
	glotheta = pose->theta;

	gomega = pose->omega;
	govel = pose->vel;

//        velocityState.SetYawRate(pose->omega);        
//        velocityState.SetVelocityX(pose->vel);
//        velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());

        // Save the values.
//	globalPoseSensor->SetGlobalPose(globalPose);
      //  bool res = localPoseSensor->SetLocalPose(globalPose);
	//std::cout << "Local Pose Res " << res << std::endl;
//        velocityStateSensor->SetVelocityState(velocityState);

}

int main(int argc, char* argv[])
{

    	ros::init(argc, argv, "jaus_case"); 
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	private_n.getParam("offset_lat", offset_lat);
	private_n.getParam("offset_long", offset_long);
	private_n.getParam("lat_conversion_to_m", lat_conversion_to_m);
	private_n.getParam("long_conversion_to_m", long_conversion_to_m);

    JAUS::Component component;
        std::cout << "Staring Case JAUS... \n";

    component.AccessControlService()->SetTimeoutPeriod(0);
    //component.TransportService()->EnableDebugMessages(true);

	//ADDING SERVICES (FAKE ROBOT)

    globalPoseSensor = new JAUS::GlobalPoseSensor();
    globalPoseSensor->SetSensorUpdateRate(50); // Updates at 50 Hz 
    component.AddService(globalPoseSensor);

    localPoseSensor = new JAUS::LocalPoseSensor();
    localPoseSensor->SetSensorUpdateRate(50); // Updates at 50 Hz 
    component.AddService(localPoseSensor);

    velocityStateSensor = new JAUS::VelocityStateSensor();
    velocityStateSensor->SetSensorUpdateRate(50); // Updates at 50 Hz 
    component.AddService(velocityStateSensor);




    // Set Identification Information.
    component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,
                                                                 "Harlie");

    if(component.Initialize(JAUS::Address(gSubsystemID, gNodeID, gComponentID)) == false)
    {
        std::cout << "Failed to Initialize Component.\n";
        return 0;
    }
    
    // INIT GLOBAL POSE
    JAUS::GlobalPose globalPose;
	    globalPose.SetLatitude(offset_lat);
	    globalPose.SetLongitude(offset_long);
	    globalPose.SetAltitude(300);
	    globalPose.SetPositionRMS(0.0);
	    globalPose.SetRoll(0.0);
	    globalPose.SetPitch(0.0);
	    globalPose.SetYaw(CxUtils::CxToRadians(0));
	    globalPose.SetAttitudeRMS(0.0);
	    globalPose.SetTimeStamp(JAUS::Time::GetUtcTime());

    // Save the data to the service.
    globalPoseSensor->SetGlobalPose(globalPose);
    localPoseSensor->SetLocalPose(globalPose);

    // Init vel
    JAUS::VelocityState velocityState;
    velocityState.SetVelocityX(0.0);
    velocityState.SetYawRate(0.0);
    velocityState.SetVelocityRMS(0.0);
    velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());
    // Save the data to the service.
    velocityStateSensor->SetVelocityState(velocityState);


    JAUS::Time::Stamp printTimeMs = 0;

    double timeDiff = 0.33;
	
    ros::Subscriber pose_sub = n.subscribe("harlie_pose", 100, callback);

    while(ros::ok())
    {
		//Adding System management thing.....
		//If Shutdown
		if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Shutdown){
		 std::cout << "\nTerminating the Program. Shutdown command receviced!\n";

		break;
		}
		//If standby
		if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Standby){
		 std::cout << "\nStandby!\n";

		
		}

	globalPose.SetLatitude(glolat);
	globalPose.SetLongitude(glolong);
	globalPose.SetYaw(glotheta);
	globalPose.SetTimeStamp(JAUS::Time::GetUtcTime());

        velocityState.SetYawRate(gomega);        
        velocityState.SetVelocityX(govel);
        velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());

        // Save the values.
	globalPoseSensor->SetGlobalPose(globalPose);
        localPoseSensor->SetLocalPose(globalPose);
        velocityStateSensor->SetVelocityState(velocityState); 	

      
        if(JAUS::Time::GetUtcTimeMs() - printTimeMs > 500)
        {
            // Print status of services.
            std::cout << "\nCASEJAUS ======================================================\n";
            component.AccessControlService()->PrintStatus(); std::cout << std::endl;
            component.ManagementService()->PrintStatus(); std::cout << std::endl;
            globalPoseSensor->PrintStatus(); std::cout << std::endl;
            localPoseSensor->PrintStatus(); std::cout << std::endl;
            velocityStateSensor->PrintStatus(); std::cout << std::endl;
            printTimeMs = JAUS::Time::GetUtcTimeMs();
        }

        CxUtils::SleepMs((unsigned int)(timeDiff*1000.0));
	ros::spinOnce();
    }

    // Shutdown 
    component.Shutdown();

    return 0;
}



