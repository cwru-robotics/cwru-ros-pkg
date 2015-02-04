/*
* Example publisher
*
* Luc Bettaieb
* bettaieb@case.edu
*/

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs

ros::Publisher pub_; //Create a 'publisher' object

int main(int argc, char** argv){
	ros::init(argc, argv, "example_publisher"); //standard ros init with node name string
	ros::NodeHandle nh; //standard ros node handle

	pub_ = nh.advertise<std_msgs::Float32>("/example_topic", 1); //setting up the publisher to publish to /example_topic with a queue size of 1
	std_msgs::Float32 number; //Setting up the number object

	number.data = 0.0; //initializing the number object's data to 0

	while(ros::ok()){ //while ROS is okay...
		pub_.publish(number); //publish the number to /example_topic

		ros::Duration(0.5).sleep(); //Wait half a second

		number.data = number.data + 1.0; // Increment the number
	}

}