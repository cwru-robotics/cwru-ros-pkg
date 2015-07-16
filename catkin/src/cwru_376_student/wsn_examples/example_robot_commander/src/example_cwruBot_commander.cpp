#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
int main(int argc, char **argv)
{
ros::init(argc,argv,"cwruBot_commander"); // name of this node 
ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
//cwruBot is expecting to receive commands on topic: cmd_vel
// ALSO, create a second topic that includes a time stamp; useful for datalogging
// commands are of type geometry_msgs/Twist, but they use only velocity in x dir and
//  yaw rate in z-dir; other 4 fields will be ignored
ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
ros::Publisher cmd_publisher2 = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);

ros::Rate sleep_timer(100); //let's make a 100Hz timer

//create a variable of type "Twist", as defined in: /opt/ros/hydro/share/geometry_msgs
// any message published on a ROS topic must have a pre-defined format, so subscribers know how to
// interpret the serialized data transmission
geometry_msgs::Twist twist_cmd;
geometry_msgs::TwistStamped twist_cmd2;
// look at the components of a message of type geometry_msgs::Twist by typing:
// rosmsg show geometry_msgs/Twist
// It has 6 fields.  Let's fill them all in with some data:
twist_cmd.linear.x = 0.0;
twist_cmd.linear.y = 0.0;
twist_cmd.linear.z = 0.0;
twist_cmd.angular.x = 0.0;
twist_cmd.angular.y = 0.0;
twist_cmd.angular.z = 0.0;

twist_cmd.linear.x = 0.4;
twist_cmd2.twist = twist_cmd; // copy the twist command into twist2 message
twist_cmd2.header.stamp = ros::Time::now(); // look up the time and put it in the header


// timer test...print out a message every 1 second
ROS_INFO("count-down");
for (int j=3;j>0;j--) {
    ROS_INFO("%d",j);
    for (int i = 0; i<100;i++)
        sleep_timer.sleep();
}

int niters = 1200; //1000 iters at 100Hz is 10 seconds;
//iteration counter; at 10ms/iteration, and 0.2m/sec, expect 2mm/iter
// should move by 2m over 10 sec
for (int i=0;i<niters;i++) {
    cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no harm done
    twist_cmd2.twist = twist_cmd; // copy the twist command into twist2 message
    twist_cmd2.header.stamp = ros::Time::now(); // look up the time and put it in the header
    cmd_publisher2.publish(twist_cmd2);
    sleep_timer.sleep(); // sleep for (remainder of) 10m
}
twist_cmd.linear.x = 0.0;
twist_cmd.angular.z = -0.314;
niters=500; // 5 sec
ROS_INFO("Time to rotate negative");
for (int i=0;i<niters;i++) {
    cmd_publisher.publish(twist_cmd); // really, should only need to publish this once, but no harm done
    twist_cmd2.twist = twist_cmd; // copy the twist command into twist2 message
    twist_cmd2.header.stamp = ros::Time::now(); // look up the time and put it in the header
    cmd_publisher2.publish(twist_cmd2);
    sleep_timer.sleep(); // sleep for (remainder of) 10m
    
}
ROS_INFO("my work here is done");
//while (ros::ok()) 
{
twist_cmd.linear.x = 0.0;
twist_cmd.angular.z = 0;
cmd_publisher.publish(twist_cmd); // and halt
twist_cmd2.twist = twist_cmd; // copy the twist command into twist2 message
twist_cmd2.header.stamp = ros::Time::now(); // look up the time and put it in the header
cmd_publisher2.publish(twist_cmd2);
}


return 0;
} 