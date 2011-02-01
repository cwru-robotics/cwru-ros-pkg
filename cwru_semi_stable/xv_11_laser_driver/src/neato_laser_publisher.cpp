#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "neato_laser_publisher");
  ros::NodeHandle n;
  
  std::string port("/dev/ttyUSB0");
  unsigned int baud_rate = 115200;
  
  boost::asio::io_service io;
  boost::asio::serial_port serial(io,port);
  
  serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  
  //boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
  
  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

  uint8_t c;
  
  uint8_t start_count = 0;
  
  while (ros::ok())
  {
    // Wait until the start sequence 0x5A, 0xA5, 0x00, 0xC0 comes around
    boost::asio::read(serial,boost::asio::buffer(&c,1));
    if(start_count == 0)
    {
      if(c == 0x5A)
      {
	start_count = 1;
      }
    } else if(start_count == 1)
    {
      if(c == 0xA5)
      {
	start_count = 2;
      }
    } else if(start_count == 2)
    {
      if(c == 0x00)
      {
	start_count = 3;
      }
    } else if(start_count == 3)
    {
      if(c == 0xC0)
      {
	start_count = 0;
	// Now that entire start sequence has been found, read in the rest of the message
	// Now read speed
	uint16_t speed;
	boost::asio::read(serial,boost::asio::buffer(&speed,2));
	
	// Read in 360*4 = 1440 chars for each point
	boost::array<uint8_t, 1440> data_packet;
	boost::asio::read(serial,boost::asio::buffer(&data_packet,1440));
	
	sensor_msgs::LaserScan msg;
	msg.header.frame_id = "neato_laser";
	msg.header.stamp = ros::Time::now();
	msg.angle_min = 0.0;
	msg.angle_max = 2.0*M_PI;
	msg.angle_increment = (2.0*M_PI/360.0);
	msg.time_increment = speed/1e8;
	msg.range_min = 0.06;
	msg.range_max = 5.0;
	msg.ranges.resize(360);
	msg.intensities.resize(360);
	
	
	for(uint16_t i = 0; i < data_packet.size(); i=i+4)
	{
	  // Four bytes per reading
	  uint8_t byte0 = data_packet[i];
	  uint8_t byte1 = data_packet[i+1];
	  uint8_t byte2 = data_packet[i+2];
	  uint8_t byte3 = data_packet[i+3];
	  // First two bits of byte1 are status flags
	  uint8_t flag1 = (byte1 & 0x80) >> 7;  // No return/max range/too low of reflectivity
	  uint8_t flag2 = (byte1 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
	  // Remaining bits are the range in mm
	  uint16_t range = ((byte1 & 0x3F)<< 8) + byte0;
	  // Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
	  uint16_t intensity = (byte3 << 8) + byte2;
	  
	  msg.ranges.push_back(range / 1000.0);
	  msg.intensities.push_back(intensity);
	  
	  //ROS_INFO("%d : %d : %.2f  : %d", flag1, flag2, range/1000.0, intensity);
	}
	
	laser_pub.publish(msg);
      }
    }
  }

  return 0;
}