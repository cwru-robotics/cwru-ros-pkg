#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <cwru_base/packets.h>
#include <cwru_base/Pose.h>
#include <cwru_base/Sonar.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


using boost::asio::ip::udp;

namespace cwru_base {
	class CrioReceiver {
		public:
			CrioReceiver();
			~CrioReceiver();
			void dispatchReceivedPacket(CRIOCommand packet);
			void handlePosePacket(CRIOPosePacket packet);
			void handleDiagnosticsPacket(CRIODiagnosticsPacket packet);
		private:
			CRIOPosePacket swapPosePacket(CRIOPosePacket& packet);
			void handleSonarPing(Sonar& ping, const float ping_value, const std::string frame_id, ros::Publisher& sonar_pub);
			ros::NodeHandle nh_;
			ros::NodeHandle priv_nh_;
			ros::Publisher estop_pub_;
			ros::Publisher pose_pub_;
			ros::Publisher flipped_pose_pub_;
			ros::Publisher sonar1_pub_;
			ros::Publisher sonar2_pub_;
			ros::Publisher sonar3_pub_;
			ros::Publisher sonar4_pub_;
			ros::Publisher sonar5_pub_;
	};

	CrioReceiver::CrioReceiver(): nh_(NULL), priv_nh_("~") {
		pose_pub_ = nh_.advertise<cwru_base::Pose>("pose",1);
		flipped_pose_pub_ = nh_.advertise<cwru_base::Pose>("flipped_pose",1);
		estop_pub_ = nh_.advertise<std_msgs::Bool>("estop_status",1,true);
		sonar1_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_1",1);
		sonar2_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_2",1);
		sonar3_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_3",1);
		sonar4_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_4",1);
		sonar5_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_5",1);
	}

	CrioReceiver::~CrioReceiver() {
	}

	void CrioReceiver::dispatchReceivedPacket(CRIOCommand packet) {
		if (packet.type == POSE_t) {
			handlePosePacket(*((CRIOPosePacket*) &packet));
		} else if (packet.type == DIAGNOSTICS_t) {
			handleDiagnosticsPacket(*((CRIODiagnosticsPacket*) &packet));
		} else {
			ROS_WARN("Unhandled packet type received: %d", packet.type);
		}	
	}

	CRIOPosePacket CrioReceiver::swapPosePacket(CRIOPosePacket& packet) {
		CRIOPosePacket swapped_packet = packet;
		swapped_packet.x = ntohl(packet.x);
		swapped_packet.y = ntohl(packet.y);
		swapped_packet.theta = ntohl(packet.theta);
		swapped_packet.vel = ntohl(packet.vel);
		swapped_packet.omega = ntohl(packet.omega);
		swapped_packet.x_variance = ntohl(packet.x_variance);
		swapped_packet.y_variance = ntohl(packet.y_variance);
		swapped_packet.theta_variance = ntohl(packet.theta_variance);
		swapped_packet.omega_variance = ntohl(packet.omega_variance);
		swapped_packet.vel_variance = ntohl(packet.vel_variance);
		swapped_packet.sonar_ping_1 = ntohl(packet.sonar_ping_1);
		swapped_packet.sonar_ping_2 = ntohl(packet.sonar_ping_2);
		swapped_packet.sonar_ping_3 = ntohl(packet.sonar_ping_3);
		swapped_packet.sonar_ping_4 = ntohl(packet.sonar_ping_4);
		swapped_packet.sonar_ping_5 = ntohl(packet.sonar_ping_5);
		return swapped_packet;
	}

	void CrioReceiver::handlePosePacket(CRIOPosePacket packet) {
		ros::Time current_time = ros::Time::now();
		CRIOPosePacket swapped_packet = swapPosePacket(packet);
		Pose p, p2;
		p.x = swapped_packet.x;
		p.y = swapped_packet.y;
		p.theta = swapped_packet.theta;
		p.vel = swapped_packet.vel;
		p.omega = swapped_packet.omega;
		p.x_var = swapped_packet.x_variance;
		p.y_var = swapped_packet.y_variance;
		p.theta_var = swapped_packet.theta_variance;
		p.vel_var = swapped_packet.vel_variance;
		p.omega_var = swapped_packet.omega_variance;
		p.header.frame_id = "crio";
		p.header.stamp = current_time;
		p2 = p;
		p2.y = -p2.y;
		p2.theta = -p2.theta;
		p2.omega = -p2.omega;
		p2.header.frame_id = "flipped_crio";
		pose_pub_.publish(p);
		flipped_pose_pub_.publish(p2);

		Sonar ping;
		ping.header.stamp = current_time;
		handleSonarPing(ping, swapped_packet.sonar_ping_1, std::string("sonar_1_link"), sonar1_pub_);
		handleSonarPing(ping, swapped_packet.sonar_ping_2, std::string("sonar_2_link"), sonar2_pub_);
		handleSonarPing(ping, swapped_packet.sonar_ping_3, std::string("sonar_3_link"), sonar3_pub_);
		handleSonarPing(ping, swapped_packet.sonar_ping_4, std::string("sonar_4_link"), sonar4_pub_);
		handleSonarPing(ping, swapped_packet.sonar_ping_5, std::string("sonar_5_link"), sonar5_pub_);
		ROS_DEBUG("Handled a Pose Packet");
	}

	void CrioReceiver::handleSonarPing(Sonar& ping, const float ping_value, const std::string frame_id, ros::Publisher& sonar_pub) {
		ping.header.frame_id = frame_id;
		ping.dist = ping_value;
		sonar_pub.publish(ping);
	}

	void CrioReceiver::handleDiagnosticsPacket(CRIODiagnosticsPacket packet) {
	}
};

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "crio_receiver");
	ros::NodeHandle nh;
	cwru_base::CrioReceiver from_crio = cwru_base::CrioReceiver();
	boost::asio::io_service io_service;
	udp::socket socket(io_service, udp::endpoint(udp::v4(), 50000));
	while (nh.ok()) {
		try {
			boost::array<cwru_base::CRIOCommand, 1> recv_buf;
			udp::endpoint remote_endpoint;
			boost::system::error_code error;

			socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

			if (error && error != boost::asio::error::message_size) {
				throw boost::system::system_error(error);
			}

			from_crio.dispatchReceivedPacket(recv_buf[0]);
		} catch (std::exception& e) {
			ROS_ERROR_STREAM("cRIO receiver threw an exception: " << e.what());
		}
	}
}
