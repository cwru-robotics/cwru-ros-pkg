#!/usr/bin/env python

# Copyright (c) 2010, Eric Perko, Jesse Fish
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

import socket
import threading

import roslib
#roslib.load_manifest('cwru_base')
import rospy
from cwru_base.msg import Pose
from cwru_base.msg import Sonar
import packets

class FromCRIO:
    def __init__(self):
        self.closeConnections = False
        self.x = 0
        self.y = 0
        self.heading = 0
        self.x_var = 0
        self.y_var = 0
        self.heading_var = 0
	self.vel = 0
	self.omega = 0
	self.vel_var = 0
	self.omega_var = 0
	self.yaw_bias = 0
	self.yaw_bias_var = 0
	self.sonar_ping_1 = 0
	self.sonar_ping_2 = 0
	self.sonar_ping_3 = 0
	self.sonar_ping_4 = 0
	self.sonar_ping_5 = 0
	self.has_data = threading.Event()

        ## UDP listening socket
        self.incomingUDP_PSO = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.incomingUDP_PSO.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.incomingUDP_PSO.bind(('', 50000)) # host is '' to bind to all addresses/interfaces

        # Start the listening threads
        ## UDP listener thread
        self.UDP_listener = threading.Thread(target = self.run)
        self.UDP_listener.start()
	self.last_time = rospy.Time.now()
	self.current_time = self.last_time

    def cleanup(self):
        self.closeConnections = True
        dummyUDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dummyUDP.sendto("CLOSE",("127.0.0.1", 50000))
        dummyUDP.close()

        self.incomingUDP_PSO.close()

    def run(self):
        while True:
	    self.has_data.clear()
            data, addr = self.incomingUDP_PSO.recvfrom(1024)
	    self.current_time = rospy.Time.now()
            # Do not continue if we are closing connections
            if self.closeConnections:
                break

            type = packets.read_packet_type(data)
            if type == 1:
                #self.status = packets.read_diagnostics_packet(data)[0]
		pass
            elif type == 0:
                self.x, self.y, self.heading, self.vel, self.omega, self.yaw_bias, self.x_var, self.y_var, self.heading_var, self.vel_var, self.omega_var, self.yaw_bias_var, self.sonar_ping_1, self.sonar_ping_2, self.sonar_ping_3, self.sonar_ping_4, self.sonar_ping_5 = packets.read_pose_packet(data)
		duration = ((self.current_time - self.last_time).to_sec())
		self.has_data.set()
	    self.last_time = self.current_time

def pose_broadcaster(fromCRIO):
    pose_pub = rospy.Publisher('pose', Pose)
    converted_pose_pub = rospy.Publisher('flipped_pose', Pose)

    sonar_pub_1 = rospy.Publisher('sonar_1', Sonar)
    sonar_pub_2 = rospy.Publisher('sonar_2', Sonar)
    sonar_pub_3 = rospy.Publisher('sonar_3', Sonar)
    sonar_pub_4 = rospy.Publisher('sonar_4', Sonar)
    sonar_pub_5 = rospy.Publisher('sonar_5', Sonar)

    while not rospy.is_shutdown():
        p = Pose(x = fromCRIO.x, y = fromCRIO.y, theta = fromCRIO.heading, x_var = fromCRIO.x_var, y_var = fromCRIO.y_var, theta_var = fromCRIO.heading_var, vel = fromCRIO.vel, omega = fromCRIO.omega, vel_var = fromCRIO.vel_var, omega_var = fromCRIO.omega_var)
        p2 = Pose(x = fromCRIO.x, y = -fromCRIO.y, theta = -fromCRIO.heading, x_var = fromCRIO.x_var, y_var = fromCRIO.y_var, theta_var = fromCRIO.heading_var, vel = fromCRIO.vel, omega = -fromCRIO.omega, vel_var = fromCRIO.vel_var, omega_var = fromCRIO.omega_var)


        p.header.stamp = rospy.Time.now()
        p2.header.stamp = p.header.stamp


        ping = Sonar()
        ping.header.stamp = fromCRIO.current_time
        handlePing(ping, 'sonar_1_link',fromCRIO.sonar_ping_1,sonar_pub_1)
        handlePing(ping, 'sonar_2_link',fromCRIO.sonar_ping_2,sonar_pub_2)
        handlePing(ping, 'sonar_3_link',fromCRIO.sonar_ping_3,sonar_pub_3)
        handlePing(ping, 'sonar_4_link',fromCRIO.sonar_ping_4,sonar_pub_4)
        handlePing(ping, 'sonar_5_link',fromCRIO.sonar_ping_5,sonar_pub_5)

        

        rospy.logdebug(p)
        pose_pub.publish(p)
        converted_pose_pub.publish(p2)

        fromCRIO.has_data.wait(10)

    fromCRIO.cleanup()

def handlePing(ping, frame_id, pingValue, sonar_pub):
    ping.header.frame_id = frame_id
    ping.dist = pingValue
    #if the ping is >0 then it has a bad value
    sonar_pub.publish(ping)

if __name__ == "__main__":
    rospy.init_node('pose_broadcaster')
    f = FromCRIO()
    try:
	pose_broadcaster(f)
    except rospy.ROSInterruptException:
	pass

