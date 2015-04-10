// IM_6DOF_example.cpp
// Wyatt Newman, based on ROS tutorial 4.2 on Interactive Markers
// example 2 differs from example 1 only in that the marker frame_ID is set to "base_link"
#include <ros/ros.h>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Point.h>


    
    
    
void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker"); // this will be the node name;

    // create an interactive marker server on the topic namespace simple_marker
    interactive_markers::InteractiveMarkerServer server("example_marker");
    // look for resulting pose messages on the topic: /simple_marker/feedback,
    // which publishes a message of type visualization_msgs/InteractiveMarkerFeedback, which
    // includes a full "pose" of the marker.
    // Coordinates of the pose are with respect to the pelvis frame

    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    // later, change the reference frame to the "map" frame
    int_marker.header.frame_id = "base_link"; ///world"; // the reference frame for pose coordinates
    int_marker.name = "des_hand_pose"; //name the marker
    int_marker.description = "Interactive Marker";

    //visualization_msgs::Marker sphere_marker; //all we need is a vertex
    //sphere_marker.type = visualization_msgs::Marker::SPHERE;     
    geometry_msgs::Point temp_point_start;
    /** specify/push-in the origin for this marker */
    temp_point_start.x = 1.5; 
    temp_point_start.y = 0.0;
    temp_point_start.z = 1;
    //sphere_marker.points.push_back(temp_point_start);
    //sphere_marker.color.r = 1.0; // make this marker red
    //sphere_marker.color.g = 0.0;
    //sphere_marker.color.b = 0.0;
    //sphere_marker.color.a = 1.0;   
     // scale the sphere size
    //sphere_marker.scale.x = 0.1;
    //sphere_marker.scale.y = 0.1;
    //sphere_marker.scale.z = 0.1;
    
  /**/
    // create an arrow marker; do this 3 times to create a triad (frame)
    visualization_msgs::Marker arrow_marker_x; //this one for the x axis
    geometry_msgs::Point temp_point;

    arrow_marker_x.type = visualization_msgs::Marker::ARROW; //CUBE; //ROS example was a CUBE; changed to ARROW
    // specify/push-in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_x.points.push_back(temp_point);
    // Specify and push in the end point for the arrow 
    temp_point = temp_point_start;
    temp_point.x = 0.2; // arrow is this long in x direction
    temp_point.y = 0.0;
    temp_point.z = 0.0;
    arrow_marker_x.points.push_back(temp_point);

    // make the arrow very thin
    arrow_marker_x.scale.x = 0.01;
    arrow_marker_x.scale.y = 0.01;
    arrow_marker_x.scale.z = 0.01;

    arrow_marker_x.color.r = 1.0; // red, for the x axis
    arrow_marker_x.color.g = 0.0;
    arrow_marker_x.color.b = 0.0;
    arrow_marker_x.color.a = 1.0;

    // do this again for the y axis:
    visualization_msgs::Marker arrow_marker_y;
    arrow_marker_y.type = visualization_msgs::Marker::ARROW; 
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_y.points.push_back(temp_point);
    // Push in the end point for the arrow 
    temp_point.x = 0.0;
    temp_point.y = 0.2; // points in the y direction
    temp_point.z = 0.0;
    arrow_marker_y.points.push_back(temp_point);

    arrow_marker_y.scale.x = 0.01;
    arrow_marker_y.scale.y = 0.01;
    arrow_marker_y.scale.z = 0.01;

    arrow_marker_y.color.r = 0.0;
    arrow_marker_y.color.g = 1.0; // color it green, for y axis
    arrow_marker_y.color.b = 0.0;
    arrow_marker_y.color.a = 1.0;

    // now the z axis
    visualization_msgs::Marker arrow_marker_z;
    arrow_marker_z.type = visualization_msgs::Marker::ARROW; //CUBE;
    // Push in the origin point for the arrow 
    temp_point.x = temp_point.y = temp_point.z = 0;
    arrow_marker_z.points.push_back(temp_point);
   // Push in the end point for the arrow 
    temp_point.x = 0.0;
    temp_point.y = 0.0;
    temp_point.z = 0.2;
    arrow_marker_z.points.push_back(temp_point);

    arrow_marker_z.scale.x = 0.01;
    arrow_marker_z.scale.y = 0.01;
    arrow_marker_z.scale.z = 0.01;

    arrow_marker_z.color.r = 0.0;
    arrow_marker_z.color.g = 0.0;
    arrow_marker_z.color.b = 1.0;
    arrow_marker_z.color.a = 1.0;
/**/
    // create a control that contains the markers
    visualization_msgs::InteractiveMarkerControl IM_control;
    IM_control.always_visible = true;
    //IM_control.markers.push_back(sphere_marker);
    
    IM_control.markers.push_back(arrow_marker_x);
    IM_control.markers.push_back(arrow_marker_y);
    IM_control.markers.push_back(arrow_marker_z);
    
    // add the control to the interactive marker
    int_marker.controls.push_back(IM_control);

    // create a control that will move the marker
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl translate_control_x;
    translate_control_x.name = "move_x";
    translate_control_x.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    /** Create the Z-Axis Control*/
    visualization_msgs::InteractiveMarkerControl translate_control_z;
    translate_control_z.name = "move_z";
    translate_control_z.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_z.orientation.x = 0; //point this in the y direction
    translate_control_z.orientation.y = 1;
    translate_control_z.orientation.z = 0;
    translate_control_z.orientation.w = 1;

    /** Create the Y-Axis Control*/
    visualization_msgs::InteractiveMarkerControl translate_control_y;
    translate_control_y.name = "move_y";
    translate_control_y.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    translate_control_y.orientation.x = 0; //point this in the y direction
    translate_control_y.orientation.y = 0;
    translate_control_y.orientation.z = 1;
    translate_control_y.orientation.w = 1;

    // add x-rotation control
  /**/
    visualization_msgs::InteractiveMarkerControl rotx_control;
    rotx_control.always_visible = true;
    rotx_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotx_control.orientation.x = 1;
    rotx_control.orientation.y = 0;
    rotx_control.orientation.z = 0;
    rotx_control.orientation.w = 1;
    rotx_control.name = "rot_x";

    // add z-rotation control
    visualization_msgs::InteractiveMarkerControl rotz_control;
    rotz_control.always_visible = true;
    rotz_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    rotz_control.orientation.x = 0;
    rotz_control.orientation.y = 1;
    rotz_control.orientation.z = 0;
    rotz_control.orientation.w = 1;
    rotz_control.name = "rot_z";

    // add y-rotation control
    visualization_msgs::InteractiveMarkerControl roty_control;
    roty_control.always_visible = true;
    roty_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    roty_control.orientation.x = 0;
    roty_control.orientation.y = 0;
    roty_control.orientation.z = 1;
    roty_control.orientation.w = 1;
    roty_control.name = "rot_y";
/**/
    // add the controls to the interactive marker
    int_marker.controls.push_back(translate_control_x);    
    int_marker.controls.push_back(translate_control_y);    
    int_marker.controls.push_back(translate_control_z);
    int_marker.controls.push_back(rotx_control);
    int_marker.controls.push_back(rotz_control);
    int_marker.controls.push_back(roty_control);
    
    /** Scale Down: this makes all of the arrows/disks for the user controls smaller than the default size */
    int_marker.scale = 0.2;
    
    //let's pre-position the marker, else it will show up at the frame origin by default
    int_marker.pose.position.x = temp_point_start.x;
    int_marker.pose.position.y = temp_point_start.y;
    int_marker.pose.position.z = temp_point_start.z;
    
    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, &processFeedback);

    // 'commit' changes and send to all clients
    server.applyChanges();
    


    // start the ROS main loop
    ROS_INFO("going into spin...");
    ros::spin();
}


