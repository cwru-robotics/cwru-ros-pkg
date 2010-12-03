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

import struct

def make_vector_driver_packet(heading, speed):
    string = struct.pack(">bxxxff", 1, heading, speed)
    return string

def make_angular_rate_driver_packet(angular_rate, speed):
    string = struct.pack(">bxxxff", 7, angular_rate, speed)
    return string

def make_waypoint_packet(x, y, heading, speed):
    string = struct.pack(">bxxxffff", 2, x, y, heading, speed)
    return string

def make_reboot_packet():
    string = struct.pack(">b", 0)
    return string

def make_start_packet():
    string = struct.pack(">b", 3)
    return string

def make_stop_packet(force):
    string = struct.pack(">bb", 4, force)
    return string

def make_querystatus_packet(verbosity):
    string = struct.pack(">bb", 5, verbosity)
    return string

def make_estop_packet():
    string = struct.pack(">b", 6)
    return string

def read_packet_type(packet):
    type = struct.unpack(">b", packet[0:1])[0]
    return type

def read_pose_packet(packet):
    ret_tuple = struct.unpack(">bxxxfffffffffffffffff", packet)[1:]
    return ret_tuple

def read_diagnostics_packet(packet):
    ret_tuple = struct.unpack(">bbbxi", packet)[1:]
    return ret_tuple
