/* Copyright (c) 2010, Eric Perko
 * All rights reserved
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <endian.h>
#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cwru_base/packets.h>
#include <cwru_base/Pose.h>
#include <cwru_base/PowerState.h>
#include <cwru_base/Sonar.h>
#include <cwru_base/cRIOSensors.h>
#include <cwru_base/NavSatFix.h>
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
      void handleGPSPacket(CRIOGPSPacket packet);
      void updateDiagnostics();
    private:
      CRIOPosePacket swapPosePacket(CRIOPosePacket& packet);
      CRIODiagnosticsPacket swapDiagnosticsPacket(CRIODiagnosticsPacket& packet);
      CRIOGPSPacket swapGPSPacket(CRIOGPSPacket& packet);
      void checkEncoderTicks(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void checkYawSensor(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void checkVoltageLevels(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void checkGPSValues(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void handleSonarPing(Sonar& ping, const float ping_value, const std::string frame_id, ros::Publisher& sonar_pub);
      float swap_float(float in);
      double swap_double(double in);
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
      ros::Publisher gps_pub_;
      ros::Publisher power_pub_;
      diagnostic_updater::Updater updater_;
      diagnostic_updater::DiagnosedPublisher<cwru_base::Pose> pose_pub_;
      diagnostic_updater::DiagnosedPublisher<cwru_base::cRIOSensors> sensor_pub_;
      double desired_pose_freq_;
      double lenc_high_warn_, lenc_low_warn_, lenc_high_err_, lenc_low_err_;
      double renc_high_warn_, renc_low_warn_, renc_high_err_, renc_low_err_;
      bool push_casters_;
      CRIODiagnosticsPacket diagnostics_info_;
      CRIOPosePacket pose_packet_;
      CRIOGPSPacket gps_packet_;
  };

  CrioReceiver::CrioReceiver(): 
    priv_nh_("~"),
    pose_pub_(nh_.advertise<cwru_base::Pose>("pose",1),
        updater_,
        diagnostic_updater::FrequencyStatusParam(&desired_pose_freq_, &desired_pose_freq_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    sensor_pub_(nh_.advertise<cwru_base::cRIOSensors>("crio_sensors",1),
        updater_,
        diagnostic_updater::FrequencyStatusParam(&desired_pose_freq_, &desired_pose_freq_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam())
  {
    priv_nh_.param("expected_pose_freq", desired_pose_freq_, 50.0);
    priv_nh_.param("push_casters", push_casters_, false);
    ros::NodeHandle encoders_nh_(priv_nh_, "encoders");
    encoders_nh_.param("lenc_high_warn", lenc_high_warn_, 15.1);
    encoders_nh_.param("lenc_low_warn", lenc_low_warn_, 14.9);
    encoders_nh_.param("lenc_high_err", lenc_high_err_, 16.);
    encoders_nh_.param("lenc_low_err", lenc_low_err_, 14.);
    encoders_nh_.param("renc_high_warn", renc_high_warn_, 15.1);
    encoders_nh_.param("renc_low_warn", renc_low_warn_, 14.9);
    encoders_nh_.param("renc_high_err", renc_high_err_, 16.);
    encoders_nh_.param("renc_low_err", renc_low_err_, 14.);
    flipped_pose_pub_ = nh_.advertise<cwru_base::Pose>("flipped_pose",1);
    estop_pub_ = nh_.advertise<std_msgs::Bool>("motors_enabled",1,true);
    sonar1_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_1",1);
    sonar2_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_2",1);
    sonar3_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_3",1);
    sonar4_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_4",1);
    sonar5_pub_ = nh_.advertise<cwru_base::Sonar>("sonar_5",1);
    gps_pub_ = nh_.advertise<cwru_base::NavSatFix>("gps_fix",1);
    power_pub_ = nh_.advertise<cwru_base::PowerState>("power_state",1);
    setupDiagnostics();
  }

  void CrioReceiver::setupDiagnostics() {
    updater_.setHardwareID("CRIO-192.168.0.100");	

    updater_.add("Encoders", this, &CrioReceiver::checkEncoderTicks);
    updater_.add("Yaw Sensor", this, &CrioReceiver::checkYawSensor);
    updater_.add("Voltages", this, &CrioReceiver::checkVoltageLevels);
    updater_.add("GPS", this, &CrioReceiver::checkGPSValues);
  }

  void CrioReceiver::updateDiagnostics() {
    updater_.update();
  }
  void CrioReceiver::checkYawSensor(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("Yaw Rate", diagnostics_info_.YawRate_mV / 1000.0); 
    stat.add("Yaw Swing", diagnostics_info_.YawSwing_mV / 1000.0); 
    stat.add("Yaw Temp", diagnostics_info_.YawTemp_mV / 1000.0); 
    stat.add("Yaw Ref", diagnostics_info_.YawRef_mV / 1000.0); 
    stat.add("Yaw Bias", pose_packet_.yaw_bias);
    stat.add("Yaw Bias Variance", pose_packet_.yaw_bias_variance);
    std::string status_msg;
    unsigned char status_lvl = diagnostic_msgs::DiagnosticStatus::OK;
    if (fabs(pose_packet_.yaw_bias) > 1.) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "Yaw sensor bias has diverged; ";
    } else if (fabs(pose_packet_.yaw_bias) > 0.4 && fabs(pose_packet_.yaw_bias) < 1.) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      status_msg += "Yaw sensor bias is outside of expected bounds; ";
    } else {
      status_msg += "Yaw sensor bias is okay; ";
    }
    if ((diagnostics_info_.YawSwing_mV / 1000.0) < 0.1) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "Yaw sensor seems disconnected; ";
    } else {
      status_msg += "Yaw sensor is connected; ";
    }
    stat.summary(status_lvl, status_msg);
  }

  void CrioReceiver::checkVoltageLevels(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("24V", diagnostics_info_.VMonitor_24V_mV / 1000.0);
    stat.add("13.8V", diagnostics_info_.VMonitor_13V_mV / 1000.0);
    stat.add("5V", diagnostics_info_.VMonitor_5V_mV / 1000.0);
    stat.add("cRIO", diagnostics_info_.VMonitor_cRIO_mV  / 1000.0);
    stat.add("eStop", diagnostics_info_.VMonitor_eStop_mV / 1000.0);
    std::string status_msg;
    unsigned char status_lvl = diagnostic_msgs::DiagnosticStatus::OK;

    if (diagnostics_info_.VMonitor_cRIO_mV < 18 * 1000.0) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "cRIO voltage below 18V; ";
    } else if (diagnostics_info_.VMonitor_cRIO_mV < 20 * 1000.0) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      status_msg += "cRIO voltage below 20V; ";
    }

    if (diagnostics_info_.VMonitor_24V_mV < 21 * 1000.0) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "24V line voltage below 21V. Charge the robot immediately; ";
    } else if (diagnostics_info_.VMonitor_24V_mV < 24 * 1000.0) {
      if (status_lvl < diagnostic_msgs::DiagnosticStatus::WARN) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      status_msg += "24V line voltage below 24V; ";
    }

    int16_t v13_diff = std::abs(diagnostics_info_.VMonitor_13V_mV - 13800);
    if (v13_diff > 1000) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "13.8V line voltage is more than 1v from 13.8V; ";
    } else if (v13_diff > 400) {
      if (status_lvl < diagnostic_msgs::DiagnosticStatus::WARN) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      status_msg += "13.8V line voltage is more than 400mv from 13.8V; ";
    }

    int16_t v5_diff = std::abs(diagnostics_info_.VMonitor_5V_mV - 5000);
    if (v5_diff > 300) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "5V line voltage is more than 300mv from 5V; ";
    } else if (v5_diff > 150) {
      if (status_lvl < diagnostic_msgs::DiagnosticStatus::WARN) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      }
      status_msg += "5V line voltage is more than 150mv from 5V; ";
    }
    stat.summary(status_lvl, status_msg);
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
      if (left_ratio < lenc_low_err_ || left_ratio > lenc_high_err_) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "Left encoder gear ratio way out of bounds; ";
      } else if (left_ratio < lenc_low_warn_ || left_ratio > lenc_high_warn_) {
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
      if (right_ratio < renc_low_err_ ||  right_ratio > renc_high_err_) {
        status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
        status_msg += "Right encoder gear ratio way out of bounds; ";
      } else if (right_ratio < renc_low_warn_ || right_ratio > renc_high_warn_) {
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

  void CrioReceiver::checkGPSValues(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    stat.add("Latitude", gps_packet_.latitude); 
    stat.add("Longitude", gps_packet_.longitude); 
    stat.add("Lat Std Dev", gps_packet_.lat_std_dev);
    stat.add("Long Std Dev", gps_packet_.long_std_dev); 
    stat.add("Solution Status", gps_packet_.solution_status);
    stat.add("Position Type", gps_packet_.position_type);
    stat.add("Differential Age", gps_packet_.differential_age);
    stat.add("Solution Age", gps_packet_.solution_age);
    //Need to cast the following... if they are uint8_t (chars) they get interpreted by the runtime monitor as characters and, when 0, can mess up the runtime_monitor
    stat.add("Satellites Tracked", (uint16_t) gps_packet_.satellites_tracked);
    stat.add("Satellites Computed", (uint16_t) gps_packet_.satellites_computed);
    std::string status_msg;
    status_msg += "No ranges specified for diagnostics checks. Values must be eyeballed";
    unsigned char status_lvl = diagnostic_msgs::DiagnosticStatus::OK;
    /*if (fabs(pose_packet_.yaw_bias) > 1.) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "Yaw sensor bias has diverged; ";
      } else if (fabs(pose_packet_.yaw_bias) > 0.4 && fabs(pose_packet_.yaw_bias) < 1.) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::WARN;
      status_msg += "Yaw sensor bias is outside of expected bounds; ";
      } else {
      status_msg += "Yaw sensor bias is okay; ";
      }
      if ((diagnostics_info_.YawSwing_mV / 1000.0) < 0.1) {
      status_lvl = diagnostic_msgs::DiagnosticStatus::ERROR;
      status_msg += "Yaw sensor seems disconnected; ";
      } else {
      status_msg += "Yaw sensor is connected; ";
      }*/
    stat.summary(status_lvl, status_msg);
  }

  CrioReceiver::~CrioReceiver() {
  }

  void CrioReceiver::dispatchReceivedPacket(CRIOCommand packet) {
    if (packet.type == POSE_t) {
      handlePosePacket(*((CRIOPosePacket*) &packet));
    } else if (packet.type == DIAGNOSTICS_t) {
      handleDiagnosticsPacket(*((CRIODiagnosticsPacket*) &packet));
    } else if (packet.type == GPS_t) {
      handleGPSPacket(*((CRIOGPSPacket*) &packet));
    } else {
      ROS_WARN("Unhandled packet type received: %d", packet.type);
    }	
  }

  float CrioReceiver::swap_float(float in) {
    uint32_t temp = *((uint32_t *)&in); 
    temp = ntohl(temp);
    return *((float *) &temp);
  }

  double CrioReceiver::swap_double(double in) {
    uint64_t temp = *((uint64_t *)&in);
    temp = be64toh(temp);
    return *((double *) &temp);
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
    swapped_packet.VMonitor_13V_mV = be16toh(packet.VMonitor_13V_mV);
    swapped_packet.VMonitor_5V_mV = be16toh(packet.VMonitor_5V_mV);
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

  CRIOGPSPacket CrioReceiver::swapGPSPacket(CRIOGPSPacket& packet) {
    CRIOGPSPacket swapped_packet = packet;
    swapped_packet.latitude = swap_double(packet.latitude);
    swapped_packet.longitude = swap_double(packet.longitude);
    swapped_packet.lat_std_dev = swap_float(packet.lat_std_dev);
    swapped_packet.long_std_dev = swap_float(packet.long_std_dev);
    swapped_packet.solution_status = be32toh(packet.solution_status);
    swapped_packet.position_type = be32toh(packet.position_type);
    swapped_packet.differential_age = swap_float(packet.differential_age);
    swapped_packet.solution_age = swap_float(packet.solution_age);
    return swapped_packet;
  }

  void CrioReceiver::handlePosePacket(CRIOPosePacket packet) {
    ros::Time current_time = ros::Time::now();
    CRIOPosePacket swapped_packet = swapPosePacket(packet);
    if (push_casters_) {
      swapped_packet.x = -swapped_packet.x;
      swapped_packet.y = -swapped_packet.y;
      swapped_packet.theta = swapped_packet.theta + M_PI;
      swapped_packet.vel = -swapped_packet.vel;
    }
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
    msg.data = !diagnostics_info_.eStopTriggered;
    estop_pub_.publish(msg);
	
	cwru_base::PowerState power_msg;
	power_msg.header.stamp = current_time;
	power_msg.header.frame_id = "crio";
	power_msg.battery_voltage = diagnostics_info_.VMonitor_24V_mV / 1000.0;
	power_msg.v13_8_voltage = diagnostics_info_.VMonitor_13V_mV / 1000.0;
	power_msg.motor_voltage = diagnostics_info_.VMonitor_eStop_mV / 1000.0;
	power_msg.cRIO_voltage = diagnostics_info_.VMonitor_cRIO_mV / 1000.0;
	power_pub_.publish(power_msg);
	
    cwru_base::cRIOSensors sensor_msg;
    sensor_msg.header.stamp = current_time;
    sensor_msg.header.frame_id = "crio";
    sensor_msg.left_wheel_encoder = diagnostics_info_.LWheelTicks;
    sensor_msg.right_wheel_encoder = diagnostics_info_.RWheelTicks;
    sensor_msg.left_motor_encoder = diagnostics_info_.LMotorTicks;
    sensor_msg.right_motor_encoder = diagnostics_info_.RMotorTicks;
    sensor_msg.yaw_rate = diagnostics_info_.YawRate_mV;
    sensor_msg.yaw_temp = diagnostics_info_.YawTemp_mV;
    sensor_msg.yaw_ref = diagnostics_info_.YawRef_mV;
    sensor_pub_.publish(sensor_msg);
  }

  void CrioReceiver::handleGPSPacket(CRIOGPSPacket packet) {
    ROS_DEBUG("Got a GPS Packet. Now broadcasting as a ROS topic");
    ros::Time current_time = ros::Time::now();
    CRIOGPSPacket swapped_packet = swapGPSPacket(packet);
    gps_packet_ = swapped_packet;

    cwru_base::NavSatFix fix_msg;
    fix_msg.header.stamp = current_time;
    fix_msg.header.frame_id = "crio_gps";
    fix_msg.status.service = cwru_base::NavSatStatus::SERVICE_GPS;
    fix_msg.position_covariance_type = cwru_base::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    fix_msg.longitude = swapped_packet.longitude;
    fix_msg.latitude = swapped_packet.latitude;

    fix_msg.position_covariance[0] = pow(swapped_packet.long_std_dev, 2.0);
    fix_msg.position_covariance[4] = pow(swapped_packet.lat_std_dev, 2.0);

    if (swapped_packet.solution_age > 2.0) {
      //Fix too old... throw it away
      fix_msg.status.status = cwru_base::NavSatStatus::STATUS_NO_FIX;
    } else if (swapped_packet.solution_status == 0) {
      //Solution has been computed
      if (swapped_packet.position_type == 16) {
        //Solution single - no DGPS corrections
        fix_msg.status.status = cwru_base::NavSatStatus::STATUS_FIX;
      } else if ((swapped_packet.position_type == 20) || (swapped_packet.position_type == 64)) {
        //Omnistar or Omnistar HP solution - we have DGPS
        fix_msg.status.status = cwru_base::NavSatStatus::STATUS_GBAS_FIX;
      } else {
        //No clue what type of solution we have, so throw it out
        fix_msg.status.status = cwru_base::NavSatStatus::STATUS_NO_FIX;
      }
    } else {
      //No solution has been computed. Throw it out.
      fix_msg.status.status = cwru_base::NavSatStatus::STATUS_NO_FIX;
    }

    gps_pub_.publish(fix_msg);
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
  ros::NodeHandle priv_nh("~");
  int timeout_val;
  priv_nh.param("socket_timeout", timeout_val, 10);
  cwru_base::CrioReceiver from_crio;
  boost::asio::io_service io_service;
  udp::socket socket(io_service, udp::endpoint(udp::v4(), 50000));
  deadline_timer deadline(io_service);
  deadline.expires_at(boost::posix_time::pos_infin);
  check_deadline(deadline, socket);
  boost::posix_time::seconds timeout(timeout_val);
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
