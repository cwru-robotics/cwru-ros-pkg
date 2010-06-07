#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class WSNSteering {
    public:
        WSNSteering();
    private:
	//callback to put odometry information into the class 
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);	

	/*The Wyatt Newman JAUSy Steering algorithm
	 * x,y in meters in ROS frame
	 * psi in rads in ROS frame, 0 points to true north
	 * v in meters/sec forwards velocity
	 * omega in rads/sec
	 */
	//void computeVelocities(double x, double y, double psi, double &v, double &omega);
	void computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double psi_des, double rho_des, double &v, double &omega);

	//last updated Odometry information
	nav_msgs::Odometry current_odom;

	//Loop rate in Hz
	double loop_rate;

	//put gains here in whatever type you need (int, double, etc) (though more descriptive names than k would make me happier)
	double k_psi,k_offset;

	
	//ROS communcators
	ros::NodeHandle nh_;
	ros::Subscriber odom_sub_;
	ros::Publisher twist_pub_;

};

WSNSteering::WSNSteering() {
    //Read parameters from the ROS parameter server, defaulting to value if the parameter is not there
   nh_.param("k_psi", k_psi, 3.0); 
   nh_.param("k_offset", k_offset, 2.0); 
   nh_.param("loop_rate", loop_rate, 20.0);

   //Subscribe to Odometry Topic
   odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry", 10, &WSNSteering::odomCallback, this); 
   
   //Setup velocity publisher
   twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

   //Setup the rate limiter
   ros::Rate rate(loop_rate);

   //temps
   geometry_msgs::Twist twist;
   double v;
   double omega;
   double x_Des,y_Des,psi_Des,rho_Des; // need to get these values from trajectory generator;
   double x_PSO,y_PSO,psi_PSO;

   x_Des = 46.14;//52.846;
   y_Des = -11.07;//-9.899;
   psi_Des = 0.0;//-0.7106; // 
   rho_Des=0; // zero curvature


   //Don't shutdown till the node shuts down
   while(ros::ok()) {
	//Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
   x_PSO = current_odom.pose.pose.position.x;
   y_PSO = current_odom.pose.pose.position.y;
   psi_PSO = tf::getYaw(current_odom.pose.pose.orientation);
   // spoof these: starting point from pit:
	computeVelocities(x_PSO,y_PSO,psi_PSO,x_Des,y_Des,psi_Des,rho_Des,v,omega);   
	
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

void WSNSteering::computeVelocities(double x_PSO, double y_PSO, double psi_PSO, double x_des, double y_des, double psi_des, double rho_des, double &v, double &omega) {
    //Wyatt put your code here. We will figure out the interface to Beom's GPS points later
	double tanVec[2],dx_vec[2],nVec[2],d;
	double deltaPsi;
	double pi=3.1415926536;

	tanVec[0]= cos(psi_des); //-sin(psi_des); // vector tangent to desired lineseg
	tanVec[1]= sin(psi_des); //cos(psi_des); 

	nVec[0]= -tanVec[1];  // normal vector of desired (directed) lineseg--points "left" of heading
    nVec[1]=  tanVec[0];
	dx_vec[0] = x_des-x_PSO;
	dx_vec[1] = y_des-y_PSO; //position error
	// d = -n'*dx_vec;
	d = -nVec[0]*dx_vec[0]-nVec[1]*dx_vec[1];
	deltaPsi = psi_PSO-psi_des;
	while (deltaPsi>pi)
		deltaPsi-=2*pi;
	while (deltaPsi< -pi)
		deltaPsi+=2*pi;
    v = 1.0; // I'll ask for 1 m/sec
	// this line should make Harlie always try to point North
    omega = -k_offset*d -k_psi*deltaPsi + v*rho_des; // estimate: if one radian off, spin at 1 rad/sec; i.e., set k about unity
	               // tested with ~low batteries; k=4 was nice
}	

void WSNSteering::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_odom = *odom;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "wsn_steering");
    WSNSteering steering;
}
