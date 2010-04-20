#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>

class BirdsEyeCalibrator {
	public:
		BirdsEyeCalibrator();
	private:
		void image_callback(const sensor_msgs::ImageConstPtr& msg);


		int32_t board_width;
		int32_t board_height;

		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_subscriber;
		image_transport::Publisher image_publisher;
};

BirdsEyeCalibrator::BirdsEyeCalibrator() : it_(nh_) {
	nh_.param("board_height", board_height, 6);
	nh_.param("board_width", board_width, 8);

	image_subscriber = it_.subscribe("image_rect", 1, &BirdsEyeCalibrator::image_callback, this);
	image_publisher = it_.advertise("plan_view", 1);
	int32_t board_n = this->board_width * this->board_height;
	int32_t corner_count = 0;
}

void BirdsEyeCalibrator::image_callback(const sensor_msgs::ImageConstPtr& msg) {
	//image_publisher.publish(msg);		
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "birds_eye_calibrator");
	BirdsEyeCalibrator calibrator;

	ros::spin();
}
