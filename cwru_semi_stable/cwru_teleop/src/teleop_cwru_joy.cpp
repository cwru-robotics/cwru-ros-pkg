/* Copyright (c) 2010, Eric Perko
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
                double delay_;
                int last_latch_button_;
                bool latch_linear_;
		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;
};

TeleopHarlie::TeleopHarlie(): 
	priv_nh_("~"),
	linear_axis_(1), 
	angular_axis_(0),
        linear_(1.2), 
	angular_(2), 
	delay_(0.0),
        last_latch_button_(0),
        latch_linear_(false)
{
    priv_nh_.param("linear_axis", linear_axis_, linear_axis_);
    priv_nh_.param("angular_axis", angular_axis_, angular_axis_);
    priv_nh_.param("linear_speed", linear_, linear_);
    priv_nh_.param("angular_speed", angular_, angular_);
    priv_nh_.param("delay", delay_, delay_);


    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 1, &TeleopHarlie::joyCallback, this);
}

void TeleopHarlie::joyCallback(const joy::Joy::ConstPtr& joy) {
	geometry_msgs::Twist twist;
        if (joy->buttons[4] != last_latch_button_ && joy->buttons[4] == 1) {
          latch_linear_ = !latch_linear_;    
        }
        last_latch_button_ = joy->buttons[4];
        if (latch_linear_) {
          twist.linear.x = linear_;
        } else {
          twist.linear.x = linear_ * joy->axes[linear_axis_];
        }
        twist.angular.z = angular_ * joy->axes[angular_axis_];
        ros::Duration(delay_).sleep();
	vel_pub_.publish(twist);
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "teleop_harlie");
    TeleopHarlie teleop_harlie;

    ros::spin();
}
