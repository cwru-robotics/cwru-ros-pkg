// marker_listener.cpp
// Wyatt Newman
// node that listens on topic "marker_listener" and adds a 
// a marker at specified point

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

visualization_msgs::Marker marker;  // instantiate a marker object
ros::Publisher vis_pub; // make this global so callback can use it


//callback to subscribe to joint state messages; copy results to global array for access by main()
void markerListenerCB(const geometry_msgs::Point& p_msg) {
    marker.points.push_back(p_msg);
    ROS_INFO("got a new point! x,y,z = %f %f %f",p_msg.x,p_msg.y,p_msg.z);
    vis_pub.publish( marker ); 
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker_placer"); // this will be the node name;
    ros::NodeHandle nh;

    ROS_INFO("hello, world");
    ros::Rate timer(4); //timer to run at 2 Hz
    // in rviz, "add" a "marker" and select this topic name: wsn_marker
    //ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "marker_publisher", 0 );   
    vis_pub = nh.advertise<visualization_msgs::Marker>( "marker_publisher", 0 );   
    ROS_INFO("setting up subscriber to marker_listener");
    ros::Subscriber sub = nh.subscribe("marker_listener", 1, markerListenerCB);
    
    // make this global;  visualization_msgs::Marker marker;  // instantiate a marker object
    //geometry_msgs::Point point;  // points will be used to specify where the markers go
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
    ros::spin(); // let the callback do all the work...
}


