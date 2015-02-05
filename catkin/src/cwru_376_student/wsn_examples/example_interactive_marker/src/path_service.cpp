// path_display.cpp
// Wyatt Newman, demo how to place markers in rviz
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>

//some globals...
geometry_msgs::Point vertex1;
geometry_msgs::Point vertex2;
geometry_msgs::Point vertex3;
geometry_msgs::Point vertex_final;
visualization_msgs::Marker marker;  // instantiate a marker object
const double dx_init_marker=0.5; // initially, assume the markers this far apart
    
void processFeedback1(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    vertex1.x = feedback->pose.position.x;
    vertex1.y = feedback->pose.position.y;
    vertex1.z = 0.1;    // place slightly above the ground level, so does not get lost in graphics
}

void processFeedback2(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    vertex2.x = feedback->pose.position.x;
    vertex2.y = feedback->pose.position.y;
    vertex2.z = 0.1;    
}

void processFeedback3(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    vertex3.x = feedback->pose.position.x;
    vertex3.y = feedback->pose.position.y;
    vertex3.z = 0.1;    
}

void processFeedbackFinalPose(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at "
            << feedback->pose.position.x << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z);
    //ROS_INFO_STREAM("reference frame is: "<<feedback->header.frame_id);
    vertex_final.x = feedback->pose.position.x;
    vertex_final.y = feedback->pose.position.y;
    vertex_final.z = 0.1;    
}

//add more points to "marker.points", starting from point1 and ending at point2
// samples spaced at intervals of ds
void add_path_points(geometry_msgs::Point point1,geometry_msgs::Point point2) {
        double ds = 0.05;
        double delta_x = point2.x - point1.x;
        double delta_y = point2.y - point1.y;
        double delta_s = sqrt(delta_x*delta_x+delta_y*delta_y);
        int npts = floor(delta_s/ds);       
        double dx = delta_x/npts;
        double dy = delta_y/npts;

        marker.points.push_back(point1);
        for (int j = 0; j < npts; j++) {
                point1.x +=dx;
                point1.y +=dy;
                marker.points.push_back(point1);
        }
        marker.points.push_back(point2);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_illustrator"); // this will be the node name;
    ros::NodeHandle nh;

    // subscribe to interactive marker coordinates
    ros::Subscriber sub1 = nh.subscribe("/vertex1/feedback", 1, processFeedback1);
    ros::Subscriber sub2 = nh.subscribe("/vertex2/feedback", 1, processFeedback2);
    ros::Subscriber sub3 = nh.subscribe("/vertex3/feedback", 1, processFeedback3);
    ros::Subscriber subFinal = nh.subscribe("/path_end/feedback", 1, processFeedbackFinalPose);
    
    // initialize assumed locations of markers (until get a callback update)
    // want to make these consistent with IM_example3 initializations;
    // really should do this by sharing a header
    vertex1.x=0.0;
    vertex2.x= vertex1.x+dx_init_marker;
    vertex3.x= vertex2.x+dx_init_marker;    
    vertex_final.x= vertex3.x+dx_init_marker; 
    vertex1.z=0.1; //elevate the path points above the floor
    vertex2.z=0.1;    
    vertex3.z=0.1;
    vertex_final.z=0.1;    
    
    ros::Rate timer(2); //timer to run at 2 Hz--update paths this fast
    // in rviz, "add" a "marker" and select this topic name: path_display
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "path_display", 0 );            

    // describe the markers to be displayed; all markers in the list will be the same style
    marker.header.frame_id = "/odom"; //base_link"; // select the reference frame 
    marker.header.stamp = ros::Time();
    marker.ns = "path_namespace";
    marker.id = 0;
    // use a SPHERE_LIST to create an array to display
    marker.type = visualization_msgs::Marker::SPHERE_LIST; //SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    //specify marker properties
    // these will all be the same for SPHERE_LIST
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // we'll make these blue

    while (ros::ok())
    {     
        marker.header.stamp = ros::Time();
        marker.points.clear(); // clear out the array
        add_path_points(vertex1,vertex2);
        add_path_points(vertex2,vertex3);     
        add_path_points(vertex3,vertex_final);          
        vis_pub.publish( marker );  //publish the new path
        ros::spinOnce(); //let callbacks perform an update
        timer.sleep();
    }
}


