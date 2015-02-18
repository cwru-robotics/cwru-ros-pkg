/* Copyright (c) 2010, Eric Perko; 2012, Edward Venator
 * All rights reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopHarlie {
	public: 
		TeleopHarlie();
	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

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
    priv_nh_.param("linear_speed_max", linear_, linear_);
    priv_nh_.param("angular_speed_max", angular_, angular_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHarlie::joyCallback, this);
}

void TeleopHarlie::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
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
