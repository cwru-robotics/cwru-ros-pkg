#include <jaus/mobility/sensors/globalposesensor.h>
#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
//#include <jaus/mobility/sensors/accelerationstatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/component.h>
#include <cxutils/keyboard.h>
#include <cxutils/math/cxmath.h>
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>

#include <tf/tf.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actoinlib/client/simpe_action_client.h>
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseActionGoal.h>


//ROS includes
#include <ros/ros.h>
#include <harlie_base/Pose.h>

#ifdef WIN32
#ifndef WIN64
//#include <vld.h>
#endif
#endif

//For GPS goal point
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
double longGoal;
double latGoal;


JAUS::UShort gSubsystemID   = 104;   // ID of our subsystem to use.
JAUS::Byte gNodeID          = 1;      // ID of our node to use.
JAUS::Byte gComponentID     = 1;      // ID of the our component.

double oriGPSLat=0;
double oriGPSLong=0;
bool issentmsgtoHarlie=false;
int serialnum =0;
bool harliesaidReached = false;

//ROS STUFF
double offset_lat;
double offset_long;
double lat_conversion_to_m;
double long_conversion_to_m;
double glolat;
double glolong;
double glotheta;
double gomega;
double govel;
//--- END


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
	if(issentmsgtoHarlie)
	{
		if (abs(glolat - latGoal) < 0.5)
		{
			if(abs(glolong - longGoal) <0.5)
			{
				harliesaidReached = true;
			}
		}
	}

//        velocityState.SetYawRate(pose->omega);
//        velocityState.SetVelocityX(pose->vel);
//        velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());

		// Save the values.
//	globalPoseSensor->SetGlobalPose(globalPose);
	  //  bool res = localPoseSensor->SetLocalPose(globalPose);
	//std::cout << "Local Pose Res " << res << std::endl;
//        velocityStateSensor->SetVelocityState(velocityState);

}


bool isrobotWaypointreached(double dstX, double dstY, double wayTolerance, double localX, double localY){
	//Get distance between Current Position to Dst Position
	double distanceAB = sqrt(pow((dstX-localX),2) + pow((dstY-localY),2));
	std::cout << "Distance LEFT...  ";
	std::cout << distanceAB;
	std::cout << "\n";
		if (distanceAB < wayTolerance){
			return true;
		}
	return false;
}

double convertLocaltoGPSLat(double dstLocalX, double oriGPSLati){

	double convertedGPSLat = oriGPSLat + (dstLocalX / 111090);
	return convertedGPSLat;

}

double convertLocaltoGPSLong(double dstLocalY, double oriGPSLongi){

	double  convertedGPSLong = oriGPSLong + (dstLocalY / 81968);

	return convertedGPSLong;
}


// PUT ROS WAYPOINT HERE!!!!!!!!!!!!!!!!!!!!
void giveROSwaypoint(int serialn,double gpsLat, double gpsLong, double speedlimit){

	Client client("move_base",true);
	client.waitForServer();
	move_base_msgs::MoveBaseGoal goal;
	longGoal=(gpaLong+83.1952306)*-1*81968;
	latGoal=(gpsLat-42.6778389)*111090;
	goal.target_pose.pose.position.x=latGoal;
	goal.target_pose.pose.position.y=longGoal;
	goal.target_pose.pose.position.z=0;
	geometry_msgs::Quaternion quad;
	quad = tf::createQuaternionMsgFromYaw(0.0);
	goal.target_pose.header.frame_id = "/odom";
	ROS_INFO("Harlie Obstacle Planner is spinning");
	goal.target_pose.pose.orientation = quad;
	client.sendGoal(goal);
	ROS_INFO("Harlie Obstacle Planner is spinning");
}


int main(int argc, char* argv[])
{
//ROS STUFF
	ros::init(argc, argv, "jaus_case");
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	private_n.getParam("offset_lat", offset_lat);
	private_n.getParam("offset_long", offset_long);
	private_n.getParam("lat_conversion_to_m", lat_conversion_to_m);
	private_n.getParam("long_conversion_to_m", long_conversion_to_m);
//--END


		JAUS::Component component;
		std::cout << "Staring Case JAUS... \n";


	component.AccessControlService()->SetTimeoutPeriod(0);


	JAUS::GlobalPoseSensor* globalPoseSensor = new JAUS::GlobalPoseSensor();
	globalPoseSensor->SetSensorUpdateRate(50); // Updates at 25 Hz
	component.AddService(globalPoseSensor);

	JAUS::LocalPoseSensor* localPoseSensor = new JAUS::LocalPoseSensor();
	localPoseSensor->SetSensorUpdateRate(50); // Updates at 25 Hz
	component.AddService(localPoseSensor);

	JAUS::VelocityStateSensor* velocityStateSensor = new JAUS::VelocityStateSensor();
	velocityStateSensor->SetSensorUpdateRate(50); // Updates at 25 Hz
	component.AddService(velocityStateSensor);

	component.AddService(new JAUS::ListManager());
	JAUS::LocalWaypointListDriver* localWaypointListDriver = new JAUS::LocalWaypointListDriver();
	component.AddService(localWaypointListDriver);

		// Try load settings files.
	if(component.LoadSettings("settings/services.xml") == false)
	{
		std::cout << "Failed to Load services.xml file! Loading default settings....\n";

	// If failed to load from file then, just load with default settings.
	component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,
																 "DEFAULT");
	} else{
	std::cout << "Loading settings.. SUCCESS!\n";
	}

	if(component.Initialize(JAUS::Address(gSubsystemID, gNodeID, gComponentID)) == false)
	{
		std::cout << "Failed to Initialize Component.\n";
		return 0;
	}


	//component.TransportService()->EnableLogging(true);

		// INIT GLOBAL POSE
	JAUS::GlobalPose globalPose;
		globalPose.SetLatitude(offset_lat);
		globalPose.SetLongitude(offset_long);
		globalPose.SetAltitude(300);
		globalPose.SetPositionRMS(0.0);
		globalPose.SetRoll(0.0);
		globalPose.SetPitch(0.0);
		globalPose.SetYaw(CxUtils::CxToRadians(45));
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

	std::vector<CxUtils::Wgs> gWaypoints;

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// FAKE SET OF WAYPOINTS

	JAUS::Address controllerID(42, 1, 1);
	component.AccessControlService()->SetControllerID(controllerID);
	component.ManagementService()->SetStatus(JAUS::Management::Status::Ready);

	JAUS::SetElement command(component.GetComponentID(), controllerID);
	command.SetRequestID(1);
	JAUS::Point3D::List localWaypoints;
	// Add more waypoints for testing as needed.
	localWaypoints.push_back(JAUS::Point3D(3, 0, 0));
	localWaypoints.push_back(JAUS::Point3D(3, 3, 0));
	localWaypoints.push_back(JAUS::Point3D(0, 3, 0));
	localWaypoints.push_back(JAUS::Point3D(0, 0, 0));

	for(unsigned int i = 0; i < (unsigned int)localWaypoints.size(); i++)
	{
		JAUS::Element e;
		// Set the element ID.
		e.mID = i + 1;
		// If this is the last element (and we are not looping) set the
		// next ID to 0, otherwise the value of the next element ID.
		if(i < (unsigned int)localWaypoints.size() - 1)
		{
			e.mNextID = e.mID + 1;
		}
		// Set previous element.
		e.mPrevID = i;
		// Populate the element message which is the command to be executed.
		JAUS::SetLocalWaypoint* message = new JAUS::SetLocalWaypoint();
		message->SetX(localWaypoints[i].mX);
		message->SetY(localWaypoints[i].mY);
		// Save the pointer
		e.mpElement = message;
		// Push completed element onto the list.
		command.GetElementList()->push_back(e);
	}

	// Set the command.
	component.TransportService()->Receive(&command);
	// Now tell it to start executing.
	JAUS::ExecuteList executeCommand(component.GetComponentID(), controllerID);
	executeCommand.SetElementUID(1); // Start at beginning.
	executeCommand.SetSpeed(1.0);    // Maximum speed.
	component.TransportService()->Receive(&executeCommand);


	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////

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

//RECEIVE WAYPOINT

		// Get local waypoint list.
		JAUS::Element::Map elementList = localWaypointListDriver->GetElementList();
		// Convert to SetLocalWaypointCommands
		JAUS::Element::Map::iterator listElement;
		std::vector<JAUS::SetLocalWaypoint> commandList;
		for(listElement = elementList.begin();
			listElement != elementList.end();
			listElement++)
		{
			if(listElement->second.mpElement->GetMessageCode() == JAUS::SET_LOCAL_WAYPOINT)
			{
				commandList.push_back(*( (JAUS::SetLocalWaypoint *)(listElement->second.mpElement)) );
			}
		}

	// std::cout << "\nWAYPOINT-----------------------------------::RECEIVED\n";

 //std::cout << localWaypointListDriver->GetmSpeed();


		//RECEIVE END


		if(JAUS::Time::GetUtcTimeMs() - printTimeMs > 500)
		{
			// Print status of services.
			std::cout << "\n======================================================\n";
			component.AccessControlService()->PrintStatus(); std::cout << std::endl;
			component.ManagementService()->PrintStatus(); std::cout << std::endl;
			globalPoseSensor->PrintStatus(); std::cout << std::endl;
			localPoseSensor->PrintStatus(); std::cout << std::endl;
			velocityStateSensor->PrintStatus(); std::cout << std::endl;
			localWaypointListDriver->PrintStatus();
			printTimeMs = JAUS::Time::GetUtcTimeMs();
		}



	//HANDLE NAV

	   if(localWaypointListDriver->IsExecuting()) // Received execute command!
	   {
			std::cout << "NAVIGATION.........\n";
		//Pull current destination X, Y from Waypoint List Driver
		double currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY, currentGloPosLat, currentGloPosLong, speedLmt;
				currentdstX =localWaypointListDriver->GetCurrentWaypoint().GetX();
				currentdstY =localWaypointListDriver->GetCurrentWaypoint().GetY();
				currentlocalX = localPoseSensor->GetLocalPose().GetX();
				currentlocalY = localPoseSensor->GetLocalPose().GetY();
				speedLmt = localWaypointListDriver->GetmSpeed();
				dstWayTolerlance = localWaypointListDriver->GetCurrentWaypoint().GetWaypointTolerance();
				std::cout << currentdstX;
				std::cout << currentdstY;

			//Check with local position to see if we have reached or not. If reached then, trigger next waypoint
				if(!isrobotWaypointreached(currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY))
			   {
				//Send signal to ROS
				if(!issentmsgtoHarlie){
					giveROSwaypoint(serialnum, convertLocaltoGPSLat(currentdstX,oriGPSLat), convertLocaltoGPSLong(currentdstY,oriGPSLong), speedLmt);
					issentmsgtoHarlie =true;
					harliesaidReached = false;
				}
				   // if already sent query, then keep driving to waypoint
			   }
			  else if (harliesaidReached && isrobotWaypointreached(currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY))
			   {
				  localWaypointListDriver->AdvanceListElement(); // Waypoint reached!  Go to next waypoint
				issentmsgtoHarlie = false;
			   }

			   if(localWaypointListDriver->GetActiveListElementID() == 0) // Reached end of list
				{
						 // Stop driving
						localWaypointListDriver->SetExecuteFlag(false); // Done!
				}
	  }else{

		oriGPSLat=globalPoseSensor->GetGlobalPose().GetLatitude();
		oriGPSLong=globalPoseSensor->GetGlobalPose().GetLongitude();


	}
	//std::cout << "NAVIGATION.........C\n";

		CxUtils::SleepMs((unsigned int)(timeDiff*1000.0));
	ros::spinOnce();
	}


	component.Shutdown();

	return 0;
}




/*  End of File */

