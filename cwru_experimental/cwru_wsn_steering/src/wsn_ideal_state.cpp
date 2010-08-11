#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <cwru_wsn_steering/DesiredState.h>
#include <cwru_wsn_steering/PathSegment.h>
#include <cwru_wsn_steering/Path.h>
#include <vector>
#include <cmath>
#include <algorithm>

class WSNIdealState {
	public:
		WSNIdealState();
	private:
		/*The Wyatt Newman JAUSy Steering algorithm
		 * x,y in meters in ROS frame
		 * psi in rads in ROS frame, 0 points to true north
		 * v in meters/sec forwards velocity
		 * omega in rads/sec
		 */
		void computeState(float& x, float& y, float& theta, float& v, float& rho);
		void pathCallback(const cwru_wsn_steering::Path::ConstPtr& p);
		void initializeDummyPath();

		//Loop rate in Hz
		double loop_rate;
		double dt;
		bool halt;

		double segDistDone;
		uint32_t iSeg;

		//Current path to be working on
		std::vector<cwru_wsn_steering::PathSegment> path;

		//ROS communcators
		ros::NodeHandle nh_;
		ros::Publisher ideal_state_pub_;
		ros::Subscriber path_sub_;
		tf::TransformListener tf_listener_;	
		geometry_msgs::PoseStamped temp_pose_in_, temp_pose_out_;
};

const double pi = acos(-1.0);

WSNIdealState::WSNIdealState() {
	//Setup the ideal state pub
	ideal_state_pub_= nh_.advertise<cwru_wsn_steering::DesiredState>("idealState",1);   
	path_sub_ = nh_.subscribe<cwru_wsn_steering::Path>("desired_path", 1, &WSNIdealState::pathCallback, this);
	nh_.param("loop_rate",loop_rate,20.0); // default 20Hz
	dt = 1.0/loop_rate;

	//Setup the rate limiter
	ros::Rate rate(loop_rate);

	//Sets up a dummy path until we get something smarter to send it down and set it
	//initializeDummyPath();

	//Initialze private class variables
	iSeg = 0;
	segDistDone = 0.0;
	halt = true;

	cwru_wsn_steering::DesiredState halt_state;
	halt_state.v = 0.0;

	//temps
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0;
	float v = 0.0;
	float rho = 0.0;

	tf_listener_.waitForTransform("odom", "map", ros::Time::now(), ros::Duration(10));
	//Don't shutdown till the node shuts down
	while(ros::ok()) {
		//Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
		computeState(x,y,theta,v,rho);

		//Put the temp vars into the desiredState
		cwru_wsn_steering::DesiredState desiredState;
		if(halt) {
			desiredState = halt_state;
		}
		else {
			desiredState.x = x;
			desiredState.y = y;
			desiredState.theta = theta;
			desiredState.v = v;
			desiredState.rho = rho;
		}
		//Publish twist message
		ideal_state_pub_.publish(desiredState);

		//Make sure this node's ROS stuff gets to run if we are hogging CPU
		ros::spinOnce();

		//Sleep till it's time to go again
		rate.sleep();
	}
}

void WSNIdealState::computeState(float& x, float& y, float& theta, float& v, float& rho)
{
	double dL = v * dt;
	if(iSeg >= path.size()) {
		//Out of bounds
		halt = true;
		v = 0.0;
		return;
	}

	segDistDone = segDistDone + dL;
	double lengthSeg = path.at(iSeg).length;
	if(segDistDone > lengthSeg) {
		segDistDone = 0.0;
		iSeg++;
	}

	if(iSeg >= path.size()) {
		//Out of bounds
		halt = true;
		v = 0.0;
		return;
	}

	cwru_wsn_steering::PathSegment currentSeg = path.at(iSeg);

	double vNext;
	v = currentSeg.vDes;
	if (iSeg < path.size()-1) {
		vNext = path.at(iSeg+1).vDes;
	} 
	else {
		vNext = 0.0;	
	}

	double tDecel = (v - vNext)/currentSeg.accel;
	double vMean = (v + vNext)/2.0;
	double distDecel = vMean*tDecel;

	double lengthRemaining = currentSeg.length - segDistDone;
	if(lengthRemaining < 0.0) {
		lengthRemaining = 0.0;
	}
	else if (lengthRemaining < distDecel) {
		v = sqrt(2*lengthRemaining*currentSeg.accel + pow(vNext, 2));
	}
	else {
		v = v + currentSeg.accel*dt;
	}

	v = std::min(v, currentSeg.vDes); //gonna fail for negative v commands along the path

	//done figuring out our velocity commands

	//Convert into the odometry frame from whatever frame the path segments are in
	temp_pose_in_.header.frame_id = currentSeg.frame_id;
	temp_pose_in_.pose.position.x = currentSeg.xRef;
	temp_pose_in_.pose.position.y = currentSeg.yRef;
	temp_pose_in_.pose.orientation = tf::createQuaternionMsgFromYaw(currentSeg.tangentAng);
	ros::Time current_transform = ros::Time::now();
	tf_listener_.getLatestCommonTime(temp_pose_in_.header.frame_id, "odom", current_transform, NULL);
	temp_pose_in_.header.stamp = current_transform;
	tf_listener_.transformPose("odom", temp_pose_in_, temp_pose_out_);

	double tanAngle = tf::getYaw(temp_pose_out_.pose.orientation);
	//std::cout << "iSeg " << iSeg << std::endl;
	//std::cout << "segDistDone " << segDistDone << std::endl;
	//std::cout << "tan angle " << tanAngle << std::endl;
	double radius, tangentAngStart, arcAngStart, dAng, arcAng;
	//std::cout << iSeg << std::endl;
	switch(currentSeg.segType){
		case 1:
			theta = tanAngle;
			rho = currentSeg.rho;
			x = temp_pose_out_.pose.position.x + segDistDone*cos(theta);
			y = temp_pose_out_.pose.position.y + segDistDone*sin(theta);
			halt = false;
			break;
		case 2:
			rho = currentSeg.rho;
			//std::cout << "rho " << rho << std::endl;
			radius = 1.0/fabs(rho);
			//std::cout << "radius = " << radius << std::endl;
			tangentAngStart = tanAngle;
			arcAngStart = 0.0;
			if(rho >= 0.0) {
				arcAngStart = tangentAngStart - pi / 2.0;	
			} else {
				arcAngStart = tangentAngStart + pi / 2.0;
			}
			dAng = segDistDone*rho;
			//std::cout << "dAng " << dAng << std::endl;
			arcAng = arcAngStart + dAng;
			x = temp_pose_out_.pose.position.x + radius * cos(arcAng);
			y = temp_pose_out_.pose.position.y  + radius * sin(arcAng);
			theta = tanAngle + dAng;
			halt = false;
			break;
		default:
			halt = true;
			v = 0.0;
	}
}

void WSNIdealState::pathCallback(const cwru_wsn_steering::Path::ConstPtr& p) {
	//Reset initial state cause the path is about to change
	iSeg = 0;
	segDistDone = 0.0;
	halt = true;   
       	path = p->segs;
}

void WSNIdealState::initializeDummyPath() {
	cwru_wsn_steering::PathSegment p;
/*	p.frame_id = "odom";
	 p.segType =1;
	   p.xRef = 0.0;
	   p.yRef = 0.0;
	   p.tangentAng = 0.0;
	   p.rho = 0.0;
	   p.length = 2.0;
	   p.vDes = 0.5;
	   p.accel = 0.1;
	   path.push_back(p);

	p.frame_id = "odom";
	   p.segType = 2;
	   p.xRef = 2.0;
	   p.yRef = 1.0;
	   p.tangentAng = 0.0;
	   p.rho = 1.0;
	   p.length = 3.1416;
	   p.vDes = 0.5;
	   p.accel = 0.1;
	   path.push_back(p);

	p.frame_id = "odom";
	   p.segType = 1;
	   p.xRef = 2.0;
	   p.yRef = 2.0;
	   p.tangentAng = 3.1416;
	   p.rho = 0.0;
	   p.length = 1.0;
	   p.vDes = 0.5;
	   p.accel = 0.1;
	   path.push_back(p);

	p.frame_id = "odom";
	   p.segType = 2;
	   p.xRef = 1.0;
	   p.yRef = 2.01;
	   p.tangentAng = 3.1416;
	   p.rho = -100.0;
	   p.length = 0.0157;
	   p.vDes = 0.01;
	   p.accel = 0.1;
	   path.push_back(p);

	p.frame_id = "odom";
	   p.segType = 1;
	   p.xRef = 0.99;
	   p.yRef = 2.01;
	   p.tangentAng = 1.5708;
	   p.rho = 0.0;
	   p.length = 1.0;
	   p.vDes = 0.5;
	   p.accel = 0.1;
	   path.push_back(p);

	p.frame_id = "odom";
	   p.segType = 2;
	   p.xRef = 1.49;
	   p.yRef = 3.01;
	   p.tangentAng = 1.5708;
	   p.rho = -2.0;
	   p.length = 0.7854;
	   p.vDes = 0.5;
	   p.accel = 0.1;
	   path.push_back(p);
	*/	   
	p.frame_id = "map";
	p.segType = 1;
	p.xRef = 0.0436;
	p.yRef = 2.18822;
	p.tangentAng = 2.42426;
	p.rho = 0.0;
	p.length = 2.3202;
	p.vDes = 0.5;
	p.accel = 0.1;
	path.push_back(p);

	p.frame_id = "map";
	p.segType = 2;
	p.xRef = -1.6972;
	p.yRef = 3.70692;
	p.tangentAng = 2.42426;
	p.rho = -100.0;
	p.length = 0.0157;
	p.vDes = 0.005;
	p.accel = 0.05;
	path.push_back(p);

	p.frame_id = "map";
	p.segType = 1;
	p.xRef = -1.7532;
	p.yRef = 3.847;
	p.tangentAng = 0.8272;
	p.rho = 0.0;
	p.length = 13.0;
	p.vDes = 0.5;
	p.accel = 0.1;
	path.push_back(p);

	p.frame_id = "map";
	p.segType = 1;
	p.xRef = 7.0274;  // start from conclusion of go-thru lab door
	p.yRef = 13.4659;
	p.tangentAng = 0.7121; //should come through door at 0.8272; recorded -0.862; vec from p1 to p2 is ang 0.7121  
	p.rho = 0.0;  // is a lineseg
	p.length = 2.1659; // lineseg length
	p.vDes = 0.5;
	p.accel = 0.1;
	path.push_back(p);

	// should end at 8.667,14.8812, 0.7121
	//      // facing elevator pose is: 8.49, 16.27, 2.375  ; should be about 90deg pos rotation
	//           // for 90-deg rotation, dist btwn pts = r*sqrt(2)
	//                // dist btwn pt 2 and pt3 = 1.40m; ==> r23 = 0.990 m; call it 1m
	//                     // rCenter is 1m to left of vector v12, from p2
	//
	p.frame_id = "map";
	p.segType = 2;
	p.xRef = 8.02;
	p.yRef = 15.63;
	p.tangentAng = 0.7121; // should agree w/ previous lineseg angle
	p.rho = 1.0; // pos rotation, CCW, w/ 1m turning radius
	p.length = 1.57; // r=1 * dtheta = pi/2 ==> pi/2
	p.vDes = 0.1;
	p.accel = 0.05;
	path.push_back(p);
	
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "wsn_ideal_state");
	WSNIdealState idealState;
}
