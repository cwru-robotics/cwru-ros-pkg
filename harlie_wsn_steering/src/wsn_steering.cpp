#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

class WSNSteering {
    public:
        WSNSteering();
    private:
	//callback to put odometry information into the class 
	void odomCallback(const nav_msgs::Odometry& odom);	

	/*The Wyatt Newman JAUSy Steering algorithm
	 * x,y in meters in ROS frame
	 * psi in rads in ROS frame, 0 points to true north
	 * v in meters/sec forwards velocity
	 * omega in rads/sec
	 */
	void computeVelocities(double x, double y, double psi, double &v, double &omega);

	//last updated Odometry information
	nav_msgs::Odometry current_odom;

	//Loop rate in Hz
	double loop_rate;

	//put gains here in whatever type you need (int, double, etc) (though more descriptive names than k would make me happier)
	double k;
	
	//ROS communcators
	ros::NodeHandle nh_;
	ros::Subscriber odom_sub_;
	ros::Publisher twist_pub_;

};

WSNSteering::WSNSteering() {
    //Read parameters from the ROS parameter server, defaulting to value if the parameter is not there
   nh_.param("k", k, 1.0); 
   nh_.param("loop_rate", loop_rate, 20.0);

   //Subscribe to Odometry Topic
   odom_sub_ = nh_.subscribe("odometry", 10, &WSNSteering::odomCallback, this); 
   
   //Setup velocity publisher
   twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

   //Setup the rate limiter
   ros::Rate rate(loop_rate);

   //temps
   geometry_msgs::Twist twist;
   double v;
   double omega;

   //Don't shutdown till the node shuts down
   while(ros::ok()) {
	//Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
	computeVelocities(current_odom.pose.pose.position.x,
	     current_odom.pose.pose.position.y,
	     tf::getYaw(current_odom.pose.pose.orientation), v, omega);	     
	printf("Post V =  %lf\nPost Omega=%lf\n",v,omega);	
	//Put values into twist message
	twist.linear.x = v;
	twist.angular.z = omega;

	//Publish twist message
	twist_pub_.publish(twist);
	
	//Make sure this node's ROS stuff gets to run if we are hogging CPU
	ros::spinOnce();

	//Sleep till it's time to go again
	rate.sleep();
   }
}

void WSNSteering::computeVelocities(double x, double y, double psi, double& v, double& omega) {
    //Wyatt put your code here. We will figure out the interface to Beom's GPS points later
    printf("x = %lf , y = %lf, psi = %lf , v = %lf, omeag = %lf\n",x,y,psi,v,omega);
    v = 0.0;
	// this line should make Harlie always try to point North
    omega = -k*psi; // estimate: if one radian off, spin at 1 rad/sec; i.e., set k about unity
    printf("Pre V =  %lf\nPre Omega=%lf\n",v,omega);	
}	

void WSNSteering::odomCallback(const nav_msgs::Odometry& odom) {
    current_odom = odom;
    printf("SHIT %lf\n",odom.pose.pose.position.y);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "wsn_steering");
    WSNSteering steering;
}
