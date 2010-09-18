#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <cwru_wsn_steering/DesiredState.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <pluginlib/class_loader.h>
#include <cwru_wsn_steering/steering_base.h>

class WSNSteering {
	public:
		WSNSteering();
		virtual ~WSNSteering();
	private:
		//callback to put odometry information into the class 
		void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);	
		void desStateCallback(const cwru_wsn_steering::DesiredState::ConstPtr& desState);

		//last updated Odometry information
		nav_msgs::Odometry current_odom;
		cwru_wsn_steering::DesiredState curDesState;
		tf::TransformListener tf_;
		costmap_2d::Costmap2DROS *local_costmap_;
		base_local_planner::TrajectoryPlannerROS planner_;

		//Loop rate in Hz
		double loop_rate;

		//put gains here in whatever type you need (int, double, etc) (though more descriptive names than k would make me happier)
		bool firstCall;
		double x_init,y_init,psi_init;

		//ROS communcators
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;
		ros::Subscriber odom_sub_;
		ros::Subscriber desState_sub_;
		ros::Publisher twist_pub_;

};


WSNSteering::WSNSteering() : priv_nh_("~") {
	//Read parameters from the ROS parameter server, defaulting to value if the parameter is not there
	bool use_collision_avoidance;
	priv_nh_.param("loop_rate", loop_rate, 20.0);
	priv_nh_.param("use_collision_avoidance", use_collision_avoidance, true);

	pluginlib::ClassLoader<cwru_wsn_steering::SteeringBase> steering_loader("cwru_wsn_steering", "cwru_wsn_steering::SteeringBase");
	cwru_wsn_steering::SteeringBase *steering_algo = NULL;

	try {
		steering_algo = steering_loader.createClassInstance("cwru_steering_algos/SecondOrderSteering");
		steering_algo->initialize(priv_nh_);

		ROS_INFO("Initialized the steering algorithm");
	} catch(pluginlib::PluginlibException& ex) {
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}

	if(use_collision_avoidance) {
		//Setup the costmap
		local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);	
		//Initialize the trajectory planner we will use for collision checking
		planner_.initialize("TrajectoryPlannerROS", &tf_, local_costmap_);
	} else {
		ROS_WARN("Collision avoidance behaviors currently disabled. The steering may issue unsafe commands");
	}

	firstCall=true;
	//Subscribe to Odometry Topic
	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odometry", 10, &WSNSteering::odomCallback, this); 
	desState_sub_ = nh_.subscribe<cwru_wsn_steering::DesiredState>("idealState", 10, &WSNSteering::desStateCallback, this);

	//Setup velocity publisher
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

	//Setup the rate limiter
	ros::Rate rate(loop_rate);

	//temps
	geometry_msgs::Twist twist;
	double v;
	double omega;
	double x_Des,y_Des,v_Des,psi_Des,rho_Des; // need to get these values from trajectory generator;
	double x_PSO,y_PSO,psi_PSO;

	//x_Des = 52.846; // set these via ideal-state generator
	//y_Des = -9.899;
	//psi_Des = -0.7106; // 
	//rho_Des=0; // zero curvature

	//Don't shutdown till the node shuts down
	while(ros::ok()) {
		if (!firstCall) // do this only when PSO is warmed up
		{
			x_Des = curDesState.x;
			y_Des = curDesState.y;
			v_Des = curDesState.v;
			psi_Des = curDesState.theta;
			rho_Des = curDesState.rho;

			//Orientation is a quaternion, so need to get yaw angle in rads.. unless you want a quaternion
			x_PSO = current_odom.pose.pose.position.x;
			y_PSO = current_odom.pose.pose.position.y;
			psi_PSO = tf::getYaw(current_odom.pose.pose.orientation);

			steering_algo->computeVelocities(x_PSO,y_PSO,psi_PSO,x_Des,y_Des,v_Des,psi_Des,rho_Des,v,omega);   

			if(use_collision_avoidance) {
				ROS_DEBUG("Using collsion avoidance behaviors");
				//check the computed velocities for safety
				if(planner_.checkTrajectory(v, 0.0, omega, true)) {
					//Legal trajectory, so we can use those values as is
					ROS_DEBUG("Legal trajectory computed... allowing v = %f, omega = %f", v, omega);
				} else {
					//Trajectory is unsafe... halt
					ROS_WARN("Unsafe speeds computed... this would have caused a collision: v = %f, omega = %f", v, omega);
					v = 0.0;
					omega = 0.0;
				}
			} else {
				ROS_DEBUG("Not using collision avoidance behaviors");
			}

			//Put values into twist message
			twist.linear.x = v;
			twist.angular.z = omega;

			//Publish twist message
			twist_pub_.publish(twist);
		}
		//Make sure this node's ROS stuff gets to run if we are hogging CPU
		ros::spinOnce();

		//Sleep till it's time to go again
		rate.sleep();
	}
}

void WSNSteering::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
	current_odom = *odom;
	if (firstCall)
	{
		firstCall=false;
		x_init=  current_odom.pose.pose.position.x;
		y_init = current_odom.pose.pose.position.y;
		psi_init = tf::getYaw(current_odom.pose.pose.orientation);
	}
}

void WSNSteering::desStateCallback(const cwru_wsn_steering::DesiredState::ConstPtr& desState)
{
	curDesState= *desState;
}

WSNSteering::~WSNSteering() {
	if(local_costmap_ != NULL) {
		delete local_costmap_;
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "wsn_steering");
	WSNSteering steering;
}
