#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest("wheelchair_base_controller")
import rospy
import math

from sensor_msgs.msg import LaserScan
import serial

class XV11_Publisher:
    def __init__(self, port):
        self.port = port
        self.pub = rospy.Publisher("neato_scan", LaserScan)

    def loop(self):
	startCount = 0
        while not rospy.is_shutdown():
	    # Hunt around for start sequence
	    if(startCount == 0): # Look for first byte
	      b = ord(port.read(1))
	      if(b == 0x5A):
		startCount = 1
	    elif(startCount == 1): # Look for second byte
	      b = ord(port.read(1))
	      if(b == 0xA5):
		startCount = 2
	    elif(startCount == 2):
	      b = ord(port.read(1))
	      if(b == 0x00):
		startCount = 3
	    elif(startCount == 3):
	      b = ord(port.read(1))
	      if(b == 0xC0):
		startCount = 0
		# We've found the entire header, time to read a message
		# Get speed bytes
		# These appear to be the time between each ping (s) * 1e8
		speedBytes = port.read(2)
		speed = (ord(speedBytes[1]) << 8) + ord(speedBytes[0])
		print speed
		
		scan = LaserScan()
		scan.header.frame_id = "neato_laser"
		scan.header.stamp = rospy.Time.now()
		scan.angle_min = 0.0
		scan.angle_max = 2.0*math.pi
		scan.angle_increment = (2.0*math.pi/360.0)
		scan.time_increment = speed/1e8#(1.0/(5.0*360.0))
		scan.range_min = 0.06
		scan.range_max = 5.0
		bytes = port.read(360*4)
		
		for i in range(360): # Go through each ping
		  # There are four bytes per point
		  byte0 = ord(bytes[i*4+0])
		  byte1 = ord(bytes[i*4+1])
		  byte2 = ord(bytes[i*4+2])
		  byte3 = ord(bytes[i*4+3])
		  # Get sensor status flag
		  flag1 = (byte1 & 0x80) >> 7
		  #print flag1
		  flag2 = (byte1 & 0x40) >> 6
		  #print flag2
		  #print "-----"
		  # Get distance
		  distStat =  ((byte1 & 0x3F)<< 8) + byte0
		  #print distStat
		  scan.ranges.append(distStat/1000.0)
		  #print byte1
		  #print byte0
		  #print "------------"
		  # Next 2 bytes are maybe intensity or uncertanty?
		  uncertanty = (byte4 << 8) + byte3
		  #print uncertanty
		  scan.intensities.append(uncertanty/1000.0)
		  
		self.pub.publish(scan)
	    
            

if __name__ == "__main__":
    rospy.init_node('neato_laser_publisher')
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
    port = serial.Serial(port_name, 115200, timeout = 1)
    broadcaster = XV11_Publisher(port)
    broadcaster.loop()
    port.close()