// IM_example2.cpp
// Wyatt Newman, based on ROS tutorial 4.2 on Interactive Markers
// create multiple (3) interactive markers; hard-coded markers--could/should be made more flexible
// topics are: vertex1, vertex2, vertex3
// also create a final marker, path_end, that includes orientation

#include <ros/ros.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>

const double dx_init_marker=0.5; // initially, space the markers this far apart
 
void processFeedback1(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
}

void processFeedback2(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    
}

void processFeedback3(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
}


void processFeedbackFinal(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("orientation: z = "<<feedback->pose.orientation.z<<"; w = "<<feedback->pose.orientation.w);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_demo"); // this will be the node name;

    // create an interactive marker server on the topic namespace vertex1
    interactive_markers::InteractiveMarkerServer server1("vertex1");
    // look for resulting pose messages on the topic: /vertex1/feedback,
    // which publishes a message of type visualization_msgs/InteractiveMarkerFeedback, which
    // includes a full "pose" of the marker.
    // Coordinates of the pose are with respect to the named frame (initially, odom--later, "map")
    interactive_markers::InteractiveMarkerServer server2("vertex2");
    interactive_markers::InteractiveMarkerServer server3("vertex3");    
    interactive_markers::InteractiveMarkerServer server_path_end("path_end");      
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    // later, change the reference frame to the "map" frame
    int_marker.header.frame_id = "/odom"; // the reference frame for pose coordinates
    int_marker.name = "vertex1"; //name the marker
    int_marker.description = "Vertex 1";

    visualization_msgs::Marker sphere_marker; //all we need is a vertex, so choose a sphere
    sphere_marker.type = visualization_msgs::Marker::SPHERE;     
    geometry_msgs::Point temp_point;
    /** specify/push-in the origin for this marker */
    temp_point.x = temp_point.y = temp_point.z = 0;
    sphere_marker.points.push_back(temp_point);
    sphere_marker.color.r = 1.0; // make this marker red
    sphere_marker.color.g = 0.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 1.0;   
     // scale the sphere size
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    
 
    // create a control that contains the marker
    visualization_msgs::InteractiveMarkerControl IM_control;
    IM_control.always_visible = true;
    IM_control.markers.push_back(sphere_marker);
    
    // add the control to the interactive marker
    int_marker.controls.push_back(IM_control);

    // create a control that will move the marker
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    // start with X-Axis Control:
    visualization_msgs::InteractiveMarkerControl translate_control_x;
    translate_control_x.name = "move_x";
    translate_control_x.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(translate_control_x);
    

    /** Create the Y-Axis Control*/
    visualization_msgs::InteractiveMarkerControl translate_control_y;
    translate_control_y.name = "move_y";
    translate_control_y.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_y.orientation.x = 0; //point this in the y direction
    translate_control_y.orientation.y = 0;
    translate_control_y.orientation.z = 1;
    translate_control_y.orientation.w = 1;
    int_marker.controls.push_back(translate_control_y);
    
    
    /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
    int_marker.scale = 0.3;
    
    //pre-position the marker
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    
    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server1.insert(int_marker, &processFeedback1);
    // 'commit' changes and send to all clients
    server1.applyChanges();
 
    // add a second marker; re-use specification, but change its name and its initial position
    int_marker.pose.position.x += dx_init_marker;
    int_marker.name = "vertex2"; //name the marker
    int_marker.description = "Vertex 2";
    // associate this marker with server2   
    server2.insert(int_marker, &processFeedback2);
    // 'commit' changes and send to all clients
    server2.applyChanges();    

    // third marker; re-use specification, but change its name and its initial position
    int_marker.pose.position.x += dx_init_marker;
    int_marker.name = "vertex3"; //name the marker
    int_marker.description = "Vertex 3";
    // associate this marker with server3   
    server3.insert(int_marker, &processFeedback3);
    // 'commit' changes and send to all clients
    server3.applyChanges();  
    
    // final marker: sphere plus heading arrow:
    int_marker.pose.position.x += dx_init_marker;
    int_marker.name = "final_pose"; //name the marker
    int_marker.description = "final pose";

    // create an arrow marker, so can display desired final heading as well
    visualization_msgs::Marker arrow_marker_x; //this one for the x axis
    arrow_marker_x.type = visualization_msgs::Marker::ARROW; //CUBE; //ROS example was a CUBE; changed to ARROW
    // specify/push-in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_x.points.push_back(temp_point);
    // Specify and push in the end point for the arrow 
    temp_point.x = 0.4; // arrow is this long in x direction
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    arrow_marker_x.points.push_back(temp_point);

    // make the arrow very thin
    arrow_marker_x.scale.x = 0.03;
    arrow_marker_x.scale.y = 0.03;
    arrow_marker_x.scale.z = 0.03;

    arrow_marker_x.color.r = 0.3; // red, for the x axis
    arrow_marker_x.color.g = 0.2;
    arrow_marker_x.color.b = 0.2;
    arrow_marker_x.color.a = 1.0;

    // create a control that contains the markers
    visualization_msgs::InteractiveMarkerControl IM_control_final;
    IM_control_final.always_visible = true;
    IM_control_final.markers.push_back(arrow_marker_x); // arrow
    IM_control_final.markers.push_back(sphere_marker);

    // add the desired controls to the interactive marker
    int_marker.controls.clear();
    int_marker.controls.push_back(IM_control_final);
    
    // add z-rotation control
    visualization_msgs::InteractiveMarkerControl rotz_control;
    rotz_control.always_visible = true;
    rotz_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotz_control.orientation.x = 0;
    rotz_control.orientation.y = 1;
    rotz_control.orientation.z = 0;
    rotz_control.orientation.w = 1;
    rotz_control.name = "rot_z";

    // add x-trans, y-trans, and z-axis rotation controls to the final interactive marker
    int_marker.controls.push_back(translate_control_x);
    int_marker.controls.push_back(translate_control_y);
    int_marker.controls.push_back(rotz_control);
    
    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_path_end.insert(int_marker, &processFeedbackFinal);

    // 'commit' changes and send to all clients
    server_path_end.applyChanges();
    
    // start the ROS main loop
    ROS_INFO("going into spin...");
    ros::spin();
}


