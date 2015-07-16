// simple_marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and prints pose received

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>


//callback to subscribe to marker state

void markerListenerCB(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_listener"); // this will be the node name;
    ros::NodeHandle nh;

  
    ROS_INFO("setting up subscriber to marker_listener");
    ros::Subscriber sub = nh.subscribe("example_marker/feedback", 1, markerListenerCB);
    ros::spin(); // let the callback do all the work...
}


