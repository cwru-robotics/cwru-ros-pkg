#!/usr/bin/python

import roslib;
import rospy
from sensor_msgs.msg import JointState

class StowedArmPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('joint_states', JointState)
    
    def loop(self):
        hz = 10
        r = rospy.Rate(hz)
        
        while not rospy.is_shutdown():
            msg = JointState();
            msg.header.stamp = rospy.Time.now()
            msg.name.append('joint1')
            msg.position.append(0.0)
            msg.name.append('joint2')
            msg.position.append(-1.57)
            msg.name.append('joint3')
            msg.position.append(0.75)
            msg.name.append('joint4')
            msg.position.append(0)
            msg.name.append('joint5')
            msg.position.append(0.03)
            msg.name.append('joint6')
            msg.position.append(-0.84)
            
            self.pub.publish(msg)
            
            r.sleep()
            
if __name__ == '__main__':
    try:
        rospy.init_node('stowed_arm_publisher')
        wsp = StowedArmPublisher()
        wsp.loop()
    except rospy.ROSInterruptException: pass
