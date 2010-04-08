#!/usr/bin/env python
import roslib
roslib.load_manifest('harlie_goal_planner')
import rospy

import tf
import actionlib

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

import yaml

class GoalReader:
    def __init__(self, filename):
	with open(filename, 'r') as filename:
	    data = yaml.safe_load(filename)
	self.goal_list = data['list']
	self.raw_goals = data['goals']
	self.loop = data['loop']
	
	self.move_base_goals = {}
	for x,y in self.raw_goals.iteritems():
	    self.move_base_goals[x] = GoalReader.create_move_base_goal_from_yaml(y)

    def create_move_base_goal_from_yaml(yaml_goal):
	"""Creates a MoveBaseGoal from a goal loaded up from the yaml file"""
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = yaml_goal['frame_id']
	goal.target_pose.header.stamp = rospy.Time.now()
	
	goal.target_pose.pose.position.x = yaml_goal['x']
	goal.target_pose.pose.position.y = yaml_goal['y']
	quaternion = tf.transformations.quaternion_about_axis(yaml_goal['theta'], (0,0,1))
	goal.target_pose.orientation = Quaternion(*quaternion)

	return goal

def main(filename):
	goal_reader = GoalReader(filename)
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	while goal_reader.loop:
	    rospy.loginfo("Looping")
	    for g in goal_reader.raw_goals:
	    	rospy.loginfo("Executing %s", g)
	    	current_goal = goal_reader.move_base_goals[g]
	    	client.send_goal(current_goal)
	    	client.wait_for_result()
	    	if client.get_state() == GoalStatus.SUCCEEDED:
	    	    rospy.loginfo("Goal executed successfully")
	    	else:
	    	    rospy.logerror("Could not execute goal for some reason")

if __name__ == '__main__':
    rospy.init_node('harlie_goal_planner')
    main('2nd_floor_goals.yaml')
