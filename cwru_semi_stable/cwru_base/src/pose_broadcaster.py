#!/usr/bin/env python

__author__="Eric Perko (exp63)"

import socket
import threading

import roslib
roslib.load_manifest('cwru_base')
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
	self.front_sonar_ping = 0
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
                self.status = packets.read_diagnostics_packet(data)[0]
            elif type == 0:
                self.x, self.y, self.heading, self.vel, self.omega, self.x_var, self.y_var, self.heading_var, self.vel_var, self.omega_var, self.front_sonar_ping = packets.read_pose_packet(data)
		duration = ((self.current_time - self.last_time).to_sec())
		self.has_data.set()
	    self.last_time = self.current_time

def pose_broadcaster(fromCRIO):
    pose_pub = rospy.Publisher('pose', Pose)
    sonar_pub = rospy.Publisher('front_sonar', Sonar)
    while not rospy.is_shutdown():
        p = Pose(x = fromCRIO.x, y = fromCRIO.y, theta = fromCRIO.heading, x_var = fromCRIO.x_var, y_var = fromCRIO.y_var, theta_var = fromCRIO.heading_var, vel = fromCRIO.vel, omega = fromCRIO.omega, vel_var = fromCRIO.vel_var, omega_var = fromCRIO.omega_var)
	ping = Sonar()
	ping.header.frame_id = 'front_sonar'
	ping.header.stamp = fromCRIO.current_time
	ping.dist = fromCRIO.front_sonar_ping
        rospy.logdebug(p)
	pose_pub.publish(p)
	sonar_pub.publish(ping)
	fromCRIO.has_data.wait(10)
    fromCRIO.cleanup()

if __name__ == "__main__":
    rospy.init_node('pose_broadcaster')
    f = FromCRIO()
    try:
	pose_broadcaster(f)
    except rospy.ROSInterruptException:
	pass

