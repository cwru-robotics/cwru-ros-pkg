#include <endian.h>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cwru_base/packets.h>
#include <cwru_base/Pose.h>
#include <cwru_base/Sonar.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

using boost::asio::ip::udp;
using boost::asio::deadline_timer;

namespace cwru_base {
  class CrioReceiver {
    public:
      CrioReceiver();
      ~CrioReceiver();
      void dispatchReceivedPacket(CRIOCommand packet);
      void handlePosePacket(CRIOPosePacket packet);
      void handleDiagnosticsPacket(CRIODiagnosticsPacket packet);
      void updateDiagnostics();
    private:
      CRIOPosePacket swapPosePacket(CRIOPosePacket& packet);
      CRIODiagnosticsPacket swapDiagnosticsPacket(CRIODiagnosticsPacket& packet);
      void checkEncoderTicks(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void handleSonarPing(Sonar& ping, const float ping_value, const std::string frame_id, ros::Publisher& sonar_pub);
      float swap_float(float in);
      void setupDiagnostics();
      ros::NodeHandle nh_;
      ros::NodeHandle priv_nh_;
      ros::Publisher estop_pub_;
      ros::Publisher flipped_pose_pub_;
      ros::Publisher sonar1_pub_;
      ros::Publisher sonar2_pub_;
      ros::Publisher sonar3_pub_;
      ros::Publisher sonar4_pub_;
      ros::Publisher sonar5_pub_;
      diagnostic_updater::Updater updater_;
      diagnostic_updater::DiagnosedPublisher<cwru_base::Pose> pose_pub_;
      double desired_pose_freq_;
      CRIODiagnosticsPacket diagnostics_info_;
      CRIOPosePacket pose_packet_;
  };

  CrioReceiver::CrioReceiver(): 
    priv_nh_("~"),
    pose_pub_(nh_.advertise<cwru_base::Pose>("pose",1),
        updater_,
        diagnostic_updater::FrequencyStatusParam(&desired_pose_freq_, &desired_pose_freq_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam())

  {
    desired_pose_freq_ = 50.0;
    flipped_pose_pub_ = nh_.advertise<cwru_base::Pose>("flipped_pose",1);
    estop_pub_ = nh_.advertise<std_msgs::Bool>("estop_status",1,true);
    sonar1_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_1",1);
    sonar2_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_2",1);
    sonar3_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_3",1);
    sonar4_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_4",1);
    sonar5_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_5",1);
    setupDiagnostics();
  }

  void CrioReceiver::setupDiagnostics() {
    updater_.setHardwareID("CRIO-192.168.0.100");	

    updater_.add("Encoders", this, &CrioReceiver::checkEncoderTicks);
  }

  void CrioReceiver::updateDiagnostics() {
    updater_.update();
  }

  void CrioReceiver::checkEncoderTicks(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    double left_ratio;
    double right_ratio;
    std::string status_msg;
    unsigned char status_lvl = diagnostic_msgs::DiagnosticStatus::OK;
    stat.add("Left Motor Ticks", diagnostics_info_.LMotorTicks);
    stat.add("Right Motor Ticks", diagnostics_info_.RMotorTicks);
    stat.add("Left Wheel Ticks", diagnostics_info_.LWheelTicks);
    stat.add("Right Wheel Ticks", diagnostics_info_.RWheelTicks);
    if (diagnostics_info_.LMotorTicks == 0 || diagnostics_info_.LWheelTicks == 0) {
      if(abs(diagnostics_info_.LMotorTicks - diagnostics_info_.LWheelTicks) > 1000) {
        //An encoder is prolly totally busted
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "One of the left encoders isn't reporting; ";
      }
      else {
        status_msg += "Both left encoders are reporting values; ";
      }
    }
    else {
      //check the ratios
      left_ratio = diagnostics_info_.LMotorTicks / ((double) diagnostics_info_.LWheelTicks);
      stat.add("Left Encoder Gear Ratio", left_ratio);
      if (left_ratio < 14 || left_ratio > 16) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "Left encoder gear ratio way out of bounds; ";
      } else if (left_ratio < 14.9 || left_ratio > 15.1) {
        if (diagnostic_msgs::DiagnosticStatus::WARN > status_lvl) {
          status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
        }
        status_msg += "Left encoder gear ratio slightly out of bounds; ";
      } else {
        status_msg += "Left encoder gear ratio okay; ";
      }
    }
    if (diagnostics_info_.RMotorTicks == 0 || diagnostics_info_.RWheelTicks == 0) {
      if(abs(diagnostics_info_.RMotorTicks - diagnostics_info_.RWheelTicks) > 1000) {
        //An encoder is prolly totally busted
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "One of the right encoders isn't reporting; ";
      }
      else {
        status_msg += "Both right encoders are reporting values; ";
      }
    } else {
      //check the ratios
      right_ratio = diagnostics_info_.RMotorTicks / (double) diagnostics_info_.RWheelTicks;
      stat.add("Right Encoder Gear Ratio", right_ratio);
      if (right_ratio < 14 ||  right_ratio > 16) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "Right encoder gear ratio way out of bounds; ";
      } else if (right_ratio < 14.9 || right_ratio > 15.1) {
        if (diagnostic_msgs::DiagnosticStatus::WARN > status_lvl) {
          status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
        }
        status_msg += "Right encoder gear ratio slightly out of bounds; ";
      } else {
        status_msg += "Right encoder gear ratio okay; ";
      }
    }

    stat.summary(status_lvl, status_msg);
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

  float CrioReceiver::swap_float(float in) {
    uint32_t temp = *((uint32_t *)&in); 
    temp = ntohl(temp);
    return *((float *) &temp);
  }

  CRIOPosePacket CrioReceiver::swapPosePacket(CRIOPosePacket& packet) {
    CRIOPosePacket swapped_packet = packet;
    swapped_packet.x = swap_float(packet.x);
    swapped_packet.y = swap_float(packet.y);
    swapped_packet.theta = swap_float(packet.theta);
    swapped_packet.vel = swap_float(packet.vel);
    swapped_packet.omega = swap_float(packet.omega);
    swapped_packet.yaw_bias = swap_float(packet.yaw_bias);
    swapped_packet.x_variance = swap_float(packet.x_variance);
    swapped_packet.y_variance = swap_float(packet.y_variance);
    swapped_packet.theta_variance = swap_float(packet.theta_variance);
    swapped_packet.omega_variance = swap_float(packet.omega_variance);
    swapped_packet.vel_variance = swap_float(packet.vel_variance);
    swapped_packet.yaw_bias_variance = swap_float(packet.yaw_bias_variance);
    swapped_packet.sonar_ping_1 = swap_float(packet.sonar_ping_1);
    swapped_packet.sonar_ping_2 = swap_float(packet.sonar_ping_2);
    swapped_packet.sonar_ping_3 = swap_float(packet.sonar_ping_3);
    swapped_packet.sonar_ping_4 = swap_float(packet.sonar_ping_4);
    swapped_packet.sonar_ping_5 = swap_float(packet.sonar_ping_5);
    return swapped_packet;
  }

  CRIODiagnosticsPacket CrioReceiver::swapDiagnosticsPacket(CRIODiagnosticsPacket& packet) {
    CRIODiagnosticsPacket swapped_packet = packet;
    swapped_packet.FPGAVersion = be16toh(packet.FPGAVersion);
    swapped_packet.VMonitor_cRIO_mV = be16toh(packet.VMonitor_cRIO_mV);
    swapped_packet.LWheelTicks = be32toh(packet.LWheelTicks);
    swapped_packet.RWheelTicks = be32toh(packet.RWheelTicks);
    swapped_packet.LMotorTicks = be32toh(packet.LMotorTicks);
    swapped_packet.RMotorTicks = be32toh(packet.RMotorTicks);
    swapped_packet.VMonitor_24V_mV = be16toh(packet.VMonitor_24V_mV);
    swapped_packet.VMonitor_13V_mv = be16toh(packet.VMonitor_13V_mv);
    swapped_packet.VMonitor_5V_mV = be16toh(packet.VMonitor_13V_mv);
    swapped_packet.VMonitor_eStop_mV = be16toh(packet.VMonitor_eStop_mV);
    swapped_packet.YawRate_mV = be16toh(packet.YawRate_mV);
    swapped_packet.YawSwing_mV = be16toh(packet.YawSwing_mV);
    swapped_packet.YawTemp_mV = be16toh(packet.YawTemp_mV);
    swapped_packet.YawRef_mV = be16toh(packet.YawRef_mV);
    swapped_packet.C1Steering = be16toh(packet.C1Steering);
    swapped_packet.C2Throttle = be16toh(packet.C2Throttle);
    swapped_packet.C3Mode = be16toh(packet.C3Mode);
    return swapped_packet;
  }

  void CrioReceiver::handlePosePacket(CRIOPosePacket packet) {
    ros::Time current_time = ros::Time::now();
    CRIOPosePacket swapped_packet = swapPosePacket(packet);
    pose_packet_ = swapped_packet;
    Pose p, p2;
    p.x = swapped_packet.x;
    p.y = swapped_packet.y;
    p.theta = swapped_packet.theta;
    p.vel = swapped_packet.vel;
    p.omega = swapped_packet.omega;
    ROS_DEBUG("Yaw bias: %f", swapped_packet.yaw_bias);
    p.x_var = swapped_packet.x_variance;
    p.y_var = swapped_packet.y_variance;
    p.theta_var = swapped_packet.theta_variance;
    p.vel_var = swapped_packet.vel_variance;
    p.omega_var = swapped_packet.omega_variance;
    ROS_DEBUG("Yaw bias variance: %f", swapped_packet.yaw_bias_variance);
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
    ros::Time current_time = ros::Time::now();
    CRIODiagnosticsPacket swapped_packet = swapDiagnosticsPacket(packet);
    diagnostics_info_ = swapped_packet;
    std_msgs::Bool msg;
    msg.data = diagnostics_info_.eStopTriggered;
    estop_pub_.publish(msg);
  }
};

void check_deadline(deadline_timer& deadline, udp::socket& socket) {
  if (deadline.expires_at() <= deadline_timer::traits_type::now()) {
    socket.cancel();
    deadline.expires_at(boost::posix_time::pos_infin);
  }
  deadline.async_wait(boost::bind(&check_deadline, boost::ref(deadline), boost::ref(socket)));
}

static void handle_receive(const boost::system::error_code& input_ec, std::size_t input_len,
    boost::system::error_code* output_ec, std::size_t* output_len) {
  *output_ec = input_ec;
  *output_len = input_len;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "crio_receiver");
  ros::NodeHandle nh;
  cwru_base::CrioReceiver from_crio;
  boost::asio::io_service io_service;
  udp::socket socket(io_service, udp::endpoint(udp::v4(), 50000));
  deadline_timer deadline(io_service);
  deadline.expires_at(boost::posix_time::pos_infin);
  check_deadline(deadline, socket);
  boost::posix_time::seconds timeout(10);
  while (nh.ok()) {
    try {
      boost::array<cwru_base::CRIOCommand, 1> recv_buf;
      udp::endpoint remote_endpoint;

      deadline.expires_from_now(timeout);

      boost::system::error_code error = boost::asio::error::would_block;
      std::size_t length = 0;

      socket.async_receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 
          boost::bind(&handle_receive, _1, _2, &error, &length));

      do {
        io_service.run_one();
      } while (error == boost::asio::error::would_block);

      if (error) {
        if (error == boost::asio::error::operation_aborted) {
          ROS_WARN("Socket receive timed out. Are you sure you are connected to the cRIO?");
          from_crio.updateDiagnostics();
        } else {
          throw boost::system::system_error(error);
        }
      } else {
        from_crio.dispatchReceivedPacket(recv_buf[0]);
        from_crio.updateDiagnostics();
      }
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("cRIO receiver threw an exception: " << e.what());
    }
  }
}
