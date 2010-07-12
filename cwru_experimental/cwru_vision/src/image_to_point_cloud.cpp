#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>

ros::Publisher pub_;
sensor_msgs::PointCloud cloud;

void image_callback(const sensor_msgs::ImageConstPtr& img);

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "image_to_point_cloud");
    ros::NodeHandle nh_;
    pub_ = nh_.advertise<sensor_msgs::PointCloud>("vision_cloud",1); 
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_subscriber = it_.subscribe("plan_view", 1, image_callback);
    ros::spin();
}

void image_callback(const sensor_msgs::ImageConstPtr& img) {
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "vision_cloud";
    geometry_msgs::Point32 temp = geometry_msgs::Point32(300*0.05, -300*0.05, 0);
    for (int i = 0; i < img->height; i++) {
	for (int j = 0; j < img->width; j++) {
	    if(img->data[image->step*i + j] > 0) {
		cloud.points << temp;
	    }
	    temp.y += 0.05
	}
	temp.x -= 0.05;
    }
}
