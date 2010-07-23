#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cwru_wsn_steering/DesiredState.h>
#include <cwru_wsn_steering/PathSegment.h>
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

};

const double pi = acos(-1.0);

WSNIdealState::WSNIdealState() {
	//Setup the ideal state pub
	ideal_state_pub_= nh_.advertise<cwru_wsn_steering::DesiredState>("idealState",1);   
	nh_.param("loop_rate",loop_rate,20.0); // default 20Hz
	dt = 1.0/loop_rate;

	//Setup the rate limiter
	ros::Rate rate(loop_rate);

	//Sets up a dummy path until we get something smarter to send it down and set it
	initializeDummyPath();

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
	segDistDone = segDistDone - lengthSeg;
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

    double radius, tangentAngStart, arcAngStart, dAng, arcAng;
    switch(currentSeg.segType){
	case 1:
	    theta = currentSeg.tangentAng;
	    rho = currentSeg.rho;
	    x = currentSeg.xRef + segDistDone*cos(theta);
	    y = currentSeg.yRef + segDistDone*sin(theta);
	    halt = false;
	    break;
	case 2:
	    rho = currentSeg.rho;
	    radius = 1.0/abs(rho);
	    tangentAngStart = currentSeg.tangentAng;
	    arcAngStart = 0.0;
	    if(rho >= 0.0) {
		arcAngStart = tangentAngStart - pi / 2.0;	
	    } else {
		arcAngStart = tangentAngStart + pi / 2.0;
	    }
	    dAng = segDistDone*rho;
	    arcAng = arcAngStart + dAng;
	    x = currentSeg.xRef + radius * cos(arcAng);
	    y = currentSeg.yRef + radius * sin(arcAng);
	    theta = currentSeg.tangentAng + dAng;
	    halt = false;
	    break;
	default:
	    halt = true;
	    v = 0.0;
    }
}

void WSNIdealState::initializeDummyPath() {
    cwru_wsn_steering::PathSegment p;
    p.segType =1;
    p.xRef = 0.0;
    p.yRef = 0.0;
    p.tangentAng = 0.0;
    p.rho = 0.0;
    p.length = 2.0;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

    p.segType = 2;
    p.xRef = 2.0;
    p.yRef = 1.0;
    p.tangentAng = 0.0;
    p.rho = 1.0;
    p.length = 3.1416;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

    p.segType = 1;
    p.xRef = 2.0;
    p.yRef = 2.0;
    p.tangentAng = 3.1416;
    p.rho = 0.0;
    p.length = 1.0;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

    p.segType = 2;
    p.xRef = 1.0;
    p.yRef = 2.01;
    p.tangentAng = 3.1416;
    p.rho = -100.0;
    p.length = 0.0157;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

    p.segType = 1;
    p.xRef = 0.99;
    p.yRef = 2.01;
    p.tangentAng = 1.5708;
    p.rho = 0.0;
    p.length = 1.0;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

    p.segType = 2;
    p.xRef = 1.49;
    p.yRef = 3.01;
    p.tangentAng = 1.5708;
    p.rho = -2.0;
    p.length = 0.7854;
    p.vDes = 0.5;
    p.accel = 1.0;
    path.push_back(p);

}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "wsn_ideal_state");
	WSNIdealState idealState;
}
