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

import roslib
roslib.load_manifest('cwru_goal_planner')
import rospy

import tf
import actionlib
import os.path
import xml.dom.minidom

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry

import yaml

class KMLGoalReader:
    def __init__(self, start_odom):
	self.offset_lat = rospy.get_param("~offset_lat")
	rospy.logdebug("offset_lat: %f",self.offset_lat)
	self.offset_long = rospy.get_param("~offset_long")
	rospy.logdebug("offset_long: %f",self.offset_long)
	self.lat_conversion_to_m = rospy.get_param("~lat_conversion_to_m")
	rospy.logdebug("lat_conversion_to_m: %f", self.lat_conversion_to_m)
	self.long_conversion_to_m = rospy.get_param("~long_conversion_to_m")
	rospy.logdebug("long_conversion_to_m: %f",self.long_conversion_to_m)
	filename = rospy.get_param("~filename")
	with open(filename, 'r') as file:
		ext = os.path.splitext(file.name)[1]
		if ext == ".kmz":
		    import zipfile
		    zipped = zipfile.ZipFile(file)
		    kml_file = zipped.open("doc.kml")
		elif ext == ".kml":
		    kml_file = file	
		else:
		    rospy.logerr("File not in kml or kmz format. Extension was %s", ext)
		tree = xml.dom.minidom.parse(kml_file)
		waypoints_list = tree.getElementsByTagName("coordinates")[0]
		waypoints_csv_list = waypoints_list.firstChild.data.strip("\n").split(" ")[:-1]
		waypoints = [waypoint.split(",")[:-1] for waypoint in waypoints_csv_list]
	self.raw_goals = waypoints
	rospy.loginfo("GPS waypoints: %s", self.raw_goals)
	
	self.move_base_goals = []
	for x in self.raw_goals:
	    self.move_base_goals.append(self.create_move_base_goal_from_gps_waypoint(x))
	rospy.loginfo("GPS waypoints in move base goals: %s", self.move_base_goals)

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = start_odom.header.frame_id
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose = start_odom.pose.pose

	rospy.loginfo("Appending start location to move base goals: %s", goal)
	self.move_base_goals.append(goal)
	rospy.loginfo("Final move base goals: %s", self.move_base_goals)

    def create_move_base_goal_from_gps_waypoint(self,waypoint):
	"""Creates a MoveBaseGoal from a goal loaded up from the yaml file"""
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'odom'
	goal.target_pose.header.stamp = rospy.Time.now()

	x = (float(waypoint[1]) - self.offset_lat) * self.lat_conversion_to_m
	y = -1 * (float(waypoint[0]) - self.offset_long) * self.long_conversion_to_m
	
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	quaternion = tf.transformations.quaternion_about_axis(0, (0,0,1))
	goal.target_pose.pose.orientation = Quaternion(*quaternion)

	return goal

class GoalReader:
	def __init__(self):
		self.odom = None
		rospy.Subscriber("odom", Odometry, self.callback)
		while not self.odom:
			pass
		rospy.loginfo("Received odom, about to start the goal planner")
		main(self.odom)
	
	def callback(self,data):
		self.odom = data

def main(odom):
	goal_reader = KMLGoalReader(odom)
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	while not rospy.is_shutdown():
	    print repr(goal_reader.raw_goals)
	    for g in goal_reader.move_base_goals:
		if rospy.is_shutdown():
		    break
	    	rospy.loginfo("Executing %s", g)
	    	client.send_goal(g)
	    	client.wait_for_result()
	    	if client.get_state() == GoalStatus.SUCCEEDED:
	    	    rospy.loginfo("Goal executed successfully")
	    	else:
	    	    rospy.logerr("Could not execute goal for some reason")
	    break

if __name__ == '__main__':
    rospy.init_node('harlie_goal_planner')
    goalReader = GoalReader()
    rospy.spin()
