//here is a simple example counterpart to "marker_listener"
// this publisher generates a bunch of points and tells the marker_listener to display them in rviz
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_marker_user_app_v2"); // name of this node will be "example_marker_user_app_v2"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<geometry_msgs::PointStamped>("marker_listener", 1);
    geometry_msgs::PointStamped stmpdPoint;
    ros::Rate timer(4); //timer to run at N Hz
    //as an example, let's build a wall of red spheres:
    double dist = 0.5; // robot distance from the wall, along x-axis in pelvis frame
    double corner_y = -0.5; // y-coordinate of right edge of the wall
    double corner_z = 0.0; // z-coordinate of lower-right corner of wall, as seen by Atlas
    stmpdPoint.header.stamp = ros::Time::now();
    stmpdPoint.header.frame_id = "utorso"; //"right_grasp_frame"; // TEST: utorso"; // hard-code this app to send pts w/rt utorso frame;
    // stmpdPoint.header.frame_id = "right_grasp_frame"; //can try alternative frames: send points, e.g., w/rt right grasp frame; displays differently
    stmpdPoint.point.x = dist;
    stmpdPoint.point.y = corner_y;
    stmpdPoint.point.z = corner_z;
    my_publisher_object.publish(stmpdPoint);
    timer.sleep();
    for (int j = 0; j < 10; j++) {

        stmpdPoint.point.y = corner_y; // start raster from y-edge
        stmpdPoint.point.x = dist; // this never changes, so don't 
        my_publisher_object.publish(stmpdPoint);
        timer.sleep();
        for (int k = 0; k < 10; k++) {
            stmpdPoint.point.y += 0.1;
            ROS_INFO("z,y = %f, %f", stmpdPoint.point.z, stmpdPoint.point.y);
            my_publisher_object.publish(stmpdPoint);
            timer.sleep();
        }
        stmpdPoint.point.z += 0.1; // next row up...
    }

}

