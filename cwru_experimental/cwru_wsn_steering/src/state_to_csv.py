#! /usr/bin/env python
import roslib
roslib.load_manifest('cwru_wsn_steering')
import rospy

from cwru_wsn_steering_msgs.msg import DesiredState

def handle_state(msg, f):
    data_string = "%.50f, %.50f\n" % (msg.x, msg.y)
    f.write(data_string)

if __name__ == "__main__":
	rospy.init_node('state_to_csv')
	with open('states.csv','w') as f:
		rospy.Subscriber('idealState', DesiredState, handle_state, f)
		rospy.spin()

