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




//ROS includes
#include <ros/ros.h>
#include <harlie_base/Pose.h>
#include <std_msgs/String.h>
#include <nav_msgs/GridCells.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <list>


#ifdef WIN32
#ifndef WIN64
//#include <vld.h>
#endif
#endif

using namespace std;

//For GPS goal point
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
double longGoal;
double latGoal;


JAUS::UShort gSubsystemID   = 104;   // ID of our subsystem to use.
JAUS::Byte gNodeID          = 1;      // ID of our node to use.
JAUS::Byte gComponentID     = 1;      // ID of the our component.


bool issentmsgtoHarlie=false;
int serialnum =0;
bool harliesaidReached = false;
bool ros_init=false;

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

int counter =0;

void callback(const harlie_base::Pose pose) {
	
	glolat = (pose.x/lat_conversion_to_m) + offset_lat;
	glolong = (pose.y/long_conversion_to_m) + offset_long;
	glotheta = pose.theta;
//	std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa\n";
//	std::cout<<glolat;
	while(glotheta>3.14159265){
glotheta -= 6.2831853;
}

while(glotheta<-3.14159265){
glotheta += 6.2831853;
}
//	cout<<glolat<<" " << glolong<< " "<<glotheta<<"\n";
	gomega = pose.omega;
	govel = pose.vel;
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
	if(!ros_init) {
		ros_init=true;
	}

}


double isrobotWaypointreached(double dstX, double dstY, double wayTolerance, double localX, double localY){
	//Get distance between Current Position to Dst Position
	double distanceAB = sqrt(pow((dstX-localX),2) + pow((dstY-localY),2));
//	std::cout << "Distance Left...  ";
//	std::cout << distanceAB;
//	std::cout << "\n";
//		if (distanceAB < wayTolerance){
//			return true;
//		}
	return distanceAB;
}

double convertLocaltoGPSLat(double dstLocalX, double dstLocalY, double oriGPSLati, double oriHeading){
// std::cout<<"ORIHEAINGGG" << oriHeading;
	dstLocalX = dstLocalX * cos(oriHeading) - dstLocalY * sin(oriHeading);
	double convertedGPSLat = oriGPSLati + (dstLocalX / 111090);
	return convertedGPSLat;
}

double convertLocaltoGPSLong(double dstLocalY, double dstLocalX,  double oriGPSLongi, double oriHeading){

	dstLocalY = dstLocalY * cos(oriHeading) + dstLocalX * sin(oriHeading);
	double  convertedGPSLong = oriGPSLongi + (dstLocalY / 81968);
	return convertedGPSLong;
}


// PUT ROS WAYPOINT HERE!!!!!!!!!!!!!!!!!!!!
void giveROSwaypoint(int serialn,double gpsLat, double gpsLong, double speedlimit){

	std::cout << "Giving ROS Waypoint.........\n"; 
//	std::cout << gpsLat;
//	std::cout << gpsLong;
//	std::cout << "..........................END\n";
	Client client("move_base",true);
	client.waitForServer();
	move_base_msgs::MoveBaseGoal goal;
	longGoal=(gpsLong+83.1952306)*-1*81968;
	latGoal=(gpsLat-42.6778389)*111090;
//	std::cout << "LongGoal \n";
//	std::cout << longGoal;
//	std::cout << "latGoal \n";
//	std::cout << latGoal;

	goal.target_pose.pose.position.x=latGoal;
	goal.target_pose.pose.position.y=longGoal;
	goal.target_pose.pose.position.z=0;
	geometry_msgs::Quaternion quad;
	quad = tf::createQuaternionMsgFromYaw(0.0);
	goal.target_pose.header.frame_id = "/odom";
	goal.target_pose.pose.orientation = quad;
	std::cout<<"Sending Goal..............";
	client.sendGoal(goal);
	std::cout<<"SentGoal\n";
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


	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	//Setting up JAUS Serivce


	JAUS::Component component;
	std::cout << "Starting Case JAUS... \n";

	ros::Subscriber pose_sub = n.subscribe("harlie_pose", 100, callback);

	//Wait until ROS responds to it
	std::cout << "Trying to connect to ROS...";
	while(!ros_init){
	std::cout << "."<< flush;
	ros::spinOnce();
	CxUtils::SleepMs((unsigned int)(500.0));
	}
	std::cout<<"CONNECTED!\n" <<flush;


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
	std::cout << "Loading settings..";
	if(component.LoadSettings("/home/harlie/jaus_case/settings/services.xml") == false)
	{
		std::cout << "..Failed to Load services.xml file!\nLoading default settings....\n";

	// If failed to load from file then, just load with default settings.
	component.DiscoveryService()->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,
																 "DEFAULT");
	} else{
	std::cout << "...SUCCESS!\n";
	}
	std::cout << "Initializing each service components...";
	if(component.Initialize(JAUS::Address(gSubsystemID, gNodeID, gComponentID)) == false)
	{
		std::cout << "...Failed to Initialize Component.\n";
		return 0;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Initialization values

	JAUS::GlobalPose globalPose;
		globalPose.SetLatitude(glolat);
		globalPose.SetLongitude(glolong);
//                globalPose.SetLatitude(42.67830);
//                globalPose.SetLongitude(-83.19510);
		globalPose.SetAltitude(0);
		globalPose.SetPositionRMS(0.0);
		globalPose.SetRoll(0.0);
		globalPose.SetPitch(0.0);
		globalPose.SetYaw(glotheta);  //??
		globalPose.SetAttitudeRMS(0.0);
		globalPose.SetTimeStamp(JAUS::Time::GetUtcTime());
	JAUS::VelocityState velocityState;
		velocityState.SetVelocityX(0.0);
		velocityState.SetYawRate(0.0);
		velocityState.SetVelocityRMS(0.0);
		velocityState.SetTimeStamp(JAUS::Time::GetUtcTime());

	std::cout<<"Initinial Value of Lat, Long, Heading from ROS is ... " << offset_lat << "," << offset_long << "," << glotheta;

	// Save the data to the service.
	globalPoseSensor->SetGlobalPose(globalPose);
	localPoseSensor->SetLocalPose(globalPose);		//<- It will automatically convert Global Coordinate in to JAUS Local Coordinate
	velocityStateSensor->SetVelocityState(velocityState);

	JAUS::Time::Stamp printTimeMs = 0;
	double timeDiff = 0.33;

	std::vector<CxUtils::Wgs> gWaypoints;

/*
	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// FAKE SET OF WAYPOINTS

	JAUS::Address controllerID(42, 1, 1);
	component.AccessControlService()->SetControllerID(controllerID);
	component.ManagementService()->SetStatus(JAUS::Management::Status::Ready);
	localPoseSensor->SetLocalPoseReference(globalPose); 
	JAUS::SetElement command(component.GetComponentID(), controllerID);
	command.SetRequestID(1);
	JAUS::Point3D::List localWaypoints;
	// Add more waypoints for testing as needed.
	localWaypoints.push_back(JAUS::Point3D(2, 0, 0));
	localWaypoints.push_back(JAUS::Point3D(2, 2, 0));
	localWaypoints.push_back(JAUS::Point3D(0, 2, 0));
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
*/

//	 ros::Subscriber pose_sub = n.subscribe("harlie_pose", 100, callback);

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Main Loop
	 while(ros::ok())
//	while(CxUtils::GetChar() != 27)
	{
		//If Shutdown
		if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Shutdown){
		 std::cout << "\nTerminating the Program. Shutdown command receviced!\n";

		break;
		}
		//If standby
		if(component.ManagementService()->GetStatus() == JAUS::Management::Status::Standby){
		 std::cout << "\nStandby!\n";
		}

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Getting data from ROS and feed that to JAUS.
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

	double oriGPSLat, oriGPSLong, tmpa, thetabefore, oriHeading;
	oriGPSLat=localPoseSensor->GetLocalPoseReference().GetLatitude();
        oriGPSLong=localPoseSensor->GetLocalPoseReference().GetLongitude();
	tmpa = localPoseSensor->GetLocalPoseReference().GetYaw();
        thetabefore =(tmpa * 3.14) / 180 ;
                if(thetabefore < 3.14){
                oriHeading = thetabefore;
                }else {oriHeading = thetabefore - 6.28; }

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// RECEIVE WAYPOINT DATA

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
	
	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Handle Waypoint Navigation 

	//for debug..
	 bool status_nav_execute = false;
	   if(localWaypointListDriver->IsExecuting()) // Received execute command!
	   {
			status_nav_execute = true;
		//Pull current destination X, Y from Waypoint List Driver
		double currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY,speedLmt;
				currentdstX =localWaypointListDriver->GetCurrentWaypoint().GetX();
				currentdstY =localWaypointListDriver->GetCurrentWaypoint().GetY();
				currentlocalX = localPoseSensor->GetLocalPose().GetX();
				currentlocalY = localPoseSensor->GetLocalPose().GetY();
				speedLmt = localWaypointListDriver->GetmSpeed();
				dstWayTolerlance = localWaypointListDriver->GetCurrentWaypoint().GetWaypointTolerance();
				std::cout << currentdstX;
				std::cout << currentdstY;

			//Check with local position to see if we have reached or not. If reached then, trigger next waypoint
				if(!(isrobotWaypointreached(currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY)<1.7))
			   {
				//Send signal to ROS
				if(!issentmsgtoHarlie){
					giveROSwaypoint(serialnum, convertLocaltoGPSLat(currentdstX, currentdstY, oriGPSLat, oriHeading), convertLocaltoGPSLong(currentdstY,currentdstX, oriGPSLong, oriHeading), speedLmt);
					issentmsgtoHarlie =true;
					harliesaidReached = false;
				}
				   // if already sent query, then keep driving to waypoint
			   }
			  else if (harliesaidReached)
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

	//	oriGPSLat=globalPoseSensor->GetGlobalPose().GetLatitude();
	//	oriGPSLong=globalPoseSensor->GetGlobalPose().GetLongitude();


	}
	//std::cout << "NAVIGATION.........C\n";

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	// Priting Status
		if(JAUS::Time::GetUtcTimeMs() - printTimeMs > 500)
		{
			// Print status of services.
			std::cout << "\nCase JAUS STATUS\n";
			std::cout << "=======================BASIC SERIVCE===============================\n";
			component.AccessControlService()->PrintStatus(); std::cout << std::endl;
			component.ManagementService()->PrintStatus(); std::cout << std::endl;
			globalPoseSensor->PrintStatus(); std::cout << std::endl;
			localPoseSensor->PrintStatus(); std::cout << std::endl;
			velocityStateSensor->PrintStatus(); std::cout << std::endl;
			localWaypointListDriver->PrintStatus();
			printTimeMs = JAUS::Time::GetUtcTimeMs();
			if(status_nav_execute){
				  double currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY;
                                currentdstX =localWaypointListDriver->GetCurrentWaypoint().GetX();
                                currentdstY =localWaypointListDriver->GetCurrentWaypoint().GetY();
                                currentlocalX = localPoseSensor->GetLocalPose().GetX();
                                currentlocalY = localPoseSensor->GetLocalPose().GetY();
                                dstWayTolerlance = localWaypointListDriver->GetCurrentWaypoint().GetWaypointTolerance();
				


			std::cout << "=======================Executing Waypoint============================\n";
			std::cout << "DISTANCE LEFT : " << isrobotWaypointreached(currentdstX, currentdstY, dstWayTolerlance, currentlocalX, currentlocalY) <<" Going to Local X,Y [" << currentdstX << ","<< currentdstY<<"] Seeding ROS GPS ["<<convertLocaltoGPSLat(currentdstX, currentdstY, oriGPSLat, oriHeading)<<","<<convertLocaltoGPSLong(currentdstY,currentdstX, oriGPSLong, oriHeading)<<"]\n";
			}
		}
/*
counter++;
std::cout<<counter;
if(counter > 3000){
std::cout<<"triggered";
        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
        // FAKE SET OF WAYPOINTS

        JAUS::Address controllerID(42, 1, 1);
        component.AccessControlService()->SetControllerID(controllerID);
        component.ManagementService()->SetStatus(JAUS::Management::Status::Ready);
        localPoseSensor->SetLocalPoseReference(globalPose);
        JAUS::SetElement command(component.GetComponentID(), controllerID);
        command.SetRequestID(1);
        JAUS::Point3D::List localWaypoints;
        // Add more waypoints for testing as needed.
        localWaypoints.push_back(JAUS::Point3D(2, 0, 0));
        localWaypoints.push_back(JAUS::Point3D(2, 2, 0));
        localWaypoints.push_back(JAUS::Point3D(0, 2, 0));
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


//       ros::Subscriber pose_sub = n.subscribe("harlie_pose", 100, callback);

        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////
}*/

		CxUtils::SleepMs((unsigned int)(timeDiff*1000.0));
	ros::spinOnce();
	}


	component.Shutdown();

	return 0;
}




/*  End of File */


