#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <iostream>

class HarlieVision {
	public:
		HarlieVision();
	private:

		void image_callback(const sensor_msgs::ImageConstPtr& msg);

		cv::Mat H;

		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
		image_transport::Publisher image_publisher;
};


HarlieVision::HarlieVision() : it_(nh_) {
	image_subscriber = it_.subscribe("image_rect_color", 1, &HarlieVision::image_callback, this);
	image_publisher = it_.advertise("plan_view", 1);

	H = (cv::Mat_<double>(3,3) << -6.5210, -1.0700e-15, 0, -0.0959, -1.5842, -0.0223, 1.9347e3, 3.489203, 1); 
}

void HarlieVision::image_callback(const sensor_msgs::ImageConstPtr& msg) {
	sensor_msgs::CvBridge bridge;
	cv::Mat image;
	cv::Mat output;
	try
	{
		image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));

	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	try {
		cv::warpPerspective(image, output, H, cv::Size(1287, 1281), CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
		cv::imshow("view", output);
		IplImage temp = output;
		image_publisher.publish(bridge.cvToImgMsg(&temp, "bgr8"));
	}
	catch (sensor_msgs::CvBridgeException& e) {

		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "harlie_vision");
	cvNamedWindow("view");
	cvStartWindowThread();
	HarlieVision visionator;
	ros::spin();
	cvDestroyWindow("view");
	return 0;
}
