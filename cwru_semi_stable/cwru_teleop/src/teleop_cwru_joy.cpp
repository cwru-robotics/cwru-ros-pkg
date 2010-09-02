#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

class TeleopHarlie {
	public: 
		TeleopHarlie();
	private:
		void joyCallback(const joy::Joy::ConstPtr& joy);

		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;

		int linear_axis_, angular_axis_;
		double linear_, angular_;
		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;
};

TeleopHarlie::TeleopHarlie(): 
	priv_nh_("~"),
	linear_(1.2), 
	angular_(2), 
	linear_axis_(1), 
	angular_axis_(0)
{
    priv_nh_.param("linear_axis", linear_axis_, linear_axis_);
    priv_nh_.param("angular_axis", angular_axis_, angular_axis_);
    priv_nh_.param("linear_", linear_, linear_);
    priv_nh_.param("angular_", angular_, angular_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopHarlie::joyCallback, this);
}

void TeleopHarlie::joyCallback(const joy::Joy::ConstPtr& joy) {
	geometry_msgs::Twist twist;
	twist.linear.x = linear_ * joy->axes[linear_axis_];
	twist.angular.z = angular_ * joy->axes[angular_axis_];
	vel_pub_.publish(twist);
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "teleop_harlie");
    TeleopHarlie teleop_harlie;

    ros::spin();
}
