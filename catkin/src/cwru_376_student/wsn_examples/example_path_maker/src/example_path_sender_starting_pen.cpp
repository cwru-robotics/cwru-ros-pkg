//example path sender
// wsn, Feb 2015
// test node compatible with example_des_state_generator.cpp;
// transmits a hard-coded path to desired-state generator node via service "appendPathService"
// the message must contain a nav_msgs/Path object

#include <stdlib.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

#include <cwru_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "cwru_srv" package
#include <cwru_srv/path_service_message.h>

int main(int argc, char **argv) {
    double dt=0.01;
    ros::init(argc, argv, "test_path_sender"); // name of this node 
    ros::NodeHandle nh; 
    ros::ServiceClient client = n.serviceClient<cwru_srv::path_service_message>("appendPathService");

    cwru_srv::path_service_message path_message;
    geometry_msgs::PoseStamped vertex;

    path_message.poses.push_back(vertex);

    // stuff the path_message, then transmit it

    if (client.call(path_message)) {
      cout<< "path server response: "<<path_message.response.resp<<endl;
        } else {
            ROS_ERROR("Failed to call service lookup_by_name");
            return 1;
        }
    return 0;
}
