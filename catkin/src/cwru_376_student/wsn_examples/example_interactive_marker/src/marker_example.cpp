// marker_example.cpp
// Wyatt Newman, demo how to place markers in rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_placer"); // this will be the node name;
    ros::NodeHandle nh;

    ROS_INFO("hello, world");
    ros::Rate timer(4); //timer to run at 2 Hz
    // in rviz, "add" a "marker" and select this topic name: wsn_marker
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "sphere_list_marker", 0 );            
    visualization_msgs::Marker marker;  // instantiate a marker object
    geometry_msgs::Point point;  // points will be used to specify where the markers go
    marker.header.frame_id = "/pelvis"; //base_link"; // select the reference frame 
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    // use SPHERE if you only want a single marker
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // if just using a single marker, specify the coordinates here, like this:

    //marker.pose.position.x = 0.4;  
    //marker.pose.position.y = -0.4;
    //marker.pose.position.z = 0;
    //ROS_INFO("x,y,z = %f %f, %f",marker.pose.position.x,marker.pose.position.y, marker.pose.position.z);    
    // otherwise, for a list of markers, put their coordinates in the "points" array, as below
    
    //whether a single marker or list of markers, need to specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    //as an example, let's build a wall of red spheres:
    double dist = 0.5; // robot distance from the wall, along x-axis in pelvis frame
    double corner_y = -0.5; // y-coordinate of right edge of the wall
    double corner_z = 0.0; // z-coordinate of lower-right corner of wall, as seen by Atlas
    point.x = dist;
    point.y = corner_y;
    point.z = corner_z;
        
    for (int j = 0; j < 10; j++) {

        point.y = corner_y;
        point.x = dist;
        marker.points.push_back(point);
        vis_pub.publish( marker );
        timer.sleep();
        for (int k = 0; k < 10; k++) {
            point.y += 0.1;
            marker.points.push_back(point);
            ROS_INFO("z,y = %f, %f",point.z,point.y);
            vis_pub.publish( marker );
             timer.sleep();
        }
        point.z += 0.1; // next row up...
    }
    
   
    for (int k=0; k<10; k++) {
        vis_pub.publish( marker );     
        timer.sleep();
        ROS_INFO("publishing...");
    }
}


