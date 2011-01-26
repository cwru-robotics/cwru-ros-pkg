#!/usr/bin/env python

import roslib; roslib.load_manifest('cwru_teleop')

import rospy
from geometry_msgs.msg import Twist

class Repeater:
    def __init__(self, rate):
        self.msg = Twist()
        self.rate = rospy.Rate(rate)
        self.pub = rospy.Publisher('output', Twist)
        self.sub = rospy.Subscriber('input', Twist, self.handle_twist)

    def handle_twist(self, twist_msg):
        self.msg = twist_msg

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("command_repeater")
    rate = rospy.get_param('~rate', 20)
    r = Repeater(rate)
    r.run()
