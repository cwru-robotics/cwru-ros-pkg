#!/usr/bin/env python

__author__="Eric Perko (exp63)"

import threading
import socket

import roslib
roslib.load_manifest('harlie_base')
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import packets


class ToCRIO:
    def __init__(self):
        """Documentation"""
        self.outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = ("192.168.0.100", 50001)

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

to_crio = ToCRIO()

def twist_receiver(msg, toCRIO):
    toCRIO.send_angular_rate_command(-1*msg.angular.z, msg.linear.x)    

def handle_reboot_request(req):
    to_crio.send_reboot_command()
    rospy.loginfo("Sent reboot command to cRIO")
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('twist_receiver')
    rospy.Subscriber('/cmd_vel', Twist, twist_receiver, to_crio)
    rospy.Service('reboot_crio', Empty, handle_reboot_request)
    rospy.spin()
        
