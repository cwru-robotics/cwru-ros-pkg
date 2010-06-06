#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <iostream>

using namespace cv;

class HarlieVision {
	public:
		HarlieVision();
	private:

		void image_callback(const sensor_msgs::ImageConstPtr& msg);
		void find_lines(const cv::Mat& input, cv::Mat& output);

		double s_c;
		double s_k;
		double v_c;
		double v_k;
		double line_threshold;
		double sub_scale;
		double black_space;

		cv::Mat H;

	    	ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
		image_transport::Publisher image_publisher;
};


HarlieVision::HarlieVision() : it_(nh_), priv_nh_("~") {
	image_subscriber = it_.subscribe("image_rect_color", 1, &HarlieVision::image_callback, this);
	image_publisher = it_.advertise("plan_view", 1);

	H = (cv::Mat_<double>(3,3) << -2.05132604, -1.09442568, 968.34979248, -2.18138890e-03, -2.11626363, 604.70727539, 1.61322518e-04, -3.26651102e-03, 1.0); 

	priv_nh_.param("s_k", s_k, 4.0);
	priv_nh_.param("s_c", s_c, 150.0);
	priv_nh_.param("v_c", v_c, 255.0);
	priv_nh_.param("v_k", v_k, 1.5);
	priv_nh_.param("line_threshold", line_threshold, 0.1);
	priv_nh_.param("sub_scale", sub_scale, 0.3);
	priv_nh_.param("black_space", black_space, 0.3);
}

void HarlieVision::image_callback(const sensor_msgs::ImageConstPtr& msg) {
	sensor_msgs::CvBridge bridge;
	cv::Mat image;
	cv::Mat output;
	cv::Mat lines;
	try
	{
		image = cv::Mat(bridge.imgMsgToCv(msg, "bgr8"));
	}
	catch (sensor_msgs::CvBridgeException& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
	try {
		find_lines(image, lines); 
		cv::warpPerspective(lines, output, H, cv::Size(600,600), CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
//		cv::imshow("view", output);
		IplImage temp = output;
		image_publisher.publish(bridge.cvToImgMsg(&temp, "mono8"));
	}
	catch (sensor_msgs::CvBridgeException& e) {

		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}

void HarlieVision::find_lines(const cv::Mat& input, cv::Mat& output) {
    Mat temp;
    Mat subtractor_layer;
    vector<Mat> mats;
    cvtColor(input, temp, CV_BGR2HSV);
    split(temp, mats);
    
    subtract(mats[1],Scalar(s_c),subtractor_layer); 
    convertScaleAbs(subtractor_layer, mats[1], s_k);
    subtract(mats[2],Scalar(v_c), subtractor_layer);
    convertScaleAbs(subtractor_layer, mats[2], v_k);

    subtract(mats[1], mats[2], mats[1]);
    subtract(mats[1], Scalar(255), subtractor_layer);
    convertScaleAbs(subtractor_layer, subtractor_layer, sub_scale);

    threshold(mats[1], mats[1], line_threshold, 0, THRESH_TOZERO);
    
    Mat roi(mats[1], Rect(0,0,mats[1].cols, mats[1].rows*black_space));    
    roi = 0.0;
    output = mats[1];
}

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "harlie_vision");
//	cvNamedWindow("view");
//	cvStartWindowThread();
	HarlieVision visionator;
	ros::spin();
//	cvDestroyWindow("view");
	return 0;
}
