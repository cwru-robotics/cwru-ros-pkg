/*
* abb_pub_base.cpp
 * wsn, March, 2015
*/

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <gazebo_msgs/ModelState.h> //message to send pose commands to gazebo
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream> //for debug I/O


int main(int argc, char** argv){
	ros::init(argc, argv, "abb_pub_base"); //standard ros init with node name string
	ros::NodeHandle nh; //standard ros node handle

	ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1); 
        
	gazebo_msgs::ModelState abb_base_state; //populate and publish this
//# Set Gazebo Model pose and twist
//string model_name           # model to set state (pose and twist)
//geometry_msgs/Pose pose     # desired pose in reference frame
//geometry_msgs/Twist twist   # desired twist in reference frame
//string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
//                            # leave empty or "world" or "map" defaults to world-frame
       
        geometry_msgs::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;        
        pose.position.z = 0.08;     
        

        
        pose.orientation.x = 0.707;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.707;         
        abb_base_state.pose = pose;
        
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;        
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;   
        abb_base_state.twist = twist;
        
        //abb_base_state.reference_frame="ground_plane";
        abb_base_state.model_name = "abbyArm";



	while(ros::ok()){ //while ROS is okay...
		pub.publish(abb_base_state); //keep republishing this
		ros::Duration(0.05).sleep(); //set the frequency
                ros::spinOnce();
	}

}