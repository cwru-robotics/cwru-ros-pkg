#!/usr/bin/env python

__author__="Eric Perko (exp63)"

import threading
import socket

import roslib
roslib.load_manifest('harlie_base')
import rospy
from geometry_msgs.msg import Twist
import packets

class ToCRIO:
    def __init__(self):
        """Documentation"""
        self.outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = ("192.168.1.100", 50001)

    def send_vector_command(self, heading, speed):
        self.outgoingUDP.sendto(packets.make_vector_driver_packet(heading, speed), self.address)

    def send_angular_rate_command(self, angular_rate, speed):
        self.outgoingUDP.sendto(packets.make_angular_rate_driver_packet(angular_rate, speed), self.address)

    def send_waypoint_command(self, x, y, heading, speed):
        self.outgoingUDP.sendto(packets.make_waypoint_packet(x, y, heading, speed), self.address)

    def send_reboot_command(self):
        self.outgoingUDP.sendto(packets.make_reboot_packet(), self.address)

    def send_start_command(self):
        self.outgoingUDP.sendto(packets.make_start_packet(), self.address)

    def send_stop_command(self, force):
        self.outgoingUDP.sendto(packets.make_stop_packet(force), self.address)

    def send_estop_command(self):
        self.outgoingUDP.sendto(packets.make_estop_packet(), self.address)

    def send_querystatus_command(self, verbosity):
        self.outgoingUDP.sendto(packets.make_querystatus_packet(verbosity), self.address)
    def cleanup(self):
	self.outgoingUDP.close()

def twist_receiver(msg, toCRIO):
    toCRIO.send_angular_rate_command(-1*msg.angular.z, msg.linear.x)    

if __name__ == "__main__":
    rospy.init_node('twist_receiver')
    s = ToCRIO()
    rospy.Subscriber('/cmd_vel', Twist, twist_receiver, s)
    rospy.spin()
        
