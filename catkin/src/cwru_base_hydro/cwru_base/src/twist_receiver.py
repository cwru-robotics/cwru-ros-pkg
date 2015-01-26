#!/usr/bin/env python

# Copyright (c) 2010, Eric Perko
# All rights reserved
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

import threading
import socket

import roslib
#roslib.load_manifest('cwru_base')
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse
import packets
import math


class ToCRIO:
    def __init__(self):
        """Documentation"""
        self.outgoingUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.address = ("192.168.0.100", 50001)
        self.ordered_halt = False

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
        self.ordered_halt = not self.ordered_halt
        self.outgoingUDP.sendto(packets.make_estop_packet(), self.address)

    def send_querystatus_command(self, verbosity):
        self.outgoingUDP.sendto(packets.make_querystatus_packet(verbosity), self.address)
    def cleanup(self):
	self.outgoingUDP.close()

to_crio = ToCRIO()

def twist_receiver(msg, params):
    toCRIO = params[0]
    push_casters = params[1]
    speed_limit = params[2]
    spin_speed_limit = params[3]
    multiplier_z = 1 if rl_swap else -1
    multiplier_x = -1 if push_casters else 1
    if abs(msg.angular.z) > spin_speed_limit:
        msg.angular.z = math.copysign(spin_speed_limit, msg.angular.z)
    if abs(msg.linear.x) > speed_limit:
        msg.linear.x = math.copysign(speed_limit, msg.linear.x)
    toCRIO.send_angular_rate_command(multiplier_z*msg.angular.z, multiplier_x*msg.linear.x)

def handle_reboot_request(req):
    to_crio.send_reboot_command()
    rospy.loginfo("Sent reboot command to cRIO")
    return EmptyResponse()

def handle_disable_motors_request(req):
    if not to_crio.ordered_halt:
        to_crio.send_estop_command()
        rospy.loginfo("Sent an estop to the cRIO to disable the motors")
    return EmptyResponse()

def handle_enable_motors_request(req):
    if to_crio.ordered_halt:
        to_crio.send_estop_command()
        rospy.loginfo("Sent an estop to the cRIO to enable the motors")
    return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node('twist_receiver')
    push_casters = rospy.get_param("~push_casters", 0)
    rl_swap = rospy.get_param("~rl_swap", 0)
    speed_limit = rospy.get_param("~speed_limit", 2.0)
    spin_speed_limit = rospy.get_param("~spin_speed_limit", 1.0)
    rospy.Subscriber('/cmd_vel', Twist, twist_receiver, (to_crio, push_casters, rl_swap, speed_limit, spin_speed_limit))
    rospy.Service('reboot_crio', Empty, handle_reboot_request)
    rospy.Service('disable_motors', Empty, handle_disable_motors_request)
    rospy.Service('enable_motors', Empty, handle_enable_motors_request)
    rospy.spin()
