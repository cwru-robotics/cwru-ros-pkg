#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <harlie_wsn_steering/DesiredState.h>

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
		void computeState(float& x, float& y, float& theta, float& rho);

		//Loop rate in Hz
		double loop_rate;

		//ROS communcators
		ros::NodeHandle nh_;
		ros::Publisher ideal_state_pub_;

};

WSNIdealState::WSNIdealState() {
	//Setup the ideal state pub
	ideal_state_pub_= nh_.advertise<harlie_wsn_steering::DesiredState>("idealState",1);   
	nh_.param("loop_rate",loop_rate,20.0); // default 20Hz
	
	//Setup the rate limiter
	ros::Rate rate(loop_rate);

	//temps
	float x;
	float y;
	float theta;
	float rho;

	//Don't shutdown till the node shuts down
	while(ros::ok()) {
		//Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
		computeState(x,y,theta,rho);

		//Put the temp vars into the desiredState
		harlie_wsn_steering::DesiredState desiredState;
		desiredState.x = x;
		desiredState.y = y;
		desiredState.theta = theta;
		desiredState.rho = rho;

		//Publish twist message
		ideal_state_pub_.publish(desiredState);

		//Make sure this node's ROS stuff gets to run if we are hogging CPU
		ros::spinOnce();

		//Sleep till it's time to go again
		rate.sleep();
	}
}
void WSNIdealState::computeState(float& x, float& y, float& theta, float& rho)
{
	//Wyatt put your code here. We will figure out the interface to Beom's GPS points later
	x = 46.14;//52.846;;
	y = -11.07;//-9.899;
	theta = 0.0;//-0.7106; //
	rho = 0;
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "wsn_ideal_state");
	WSNIdealState idealState;
}
