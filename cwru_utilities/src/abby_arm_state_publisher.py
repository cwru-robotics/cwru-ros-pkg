#!/usr/bin/python

import roslib;
import rospy
from sensor_msgs.msg import JointState

class ArmStatePublisher():
    def __init__(self):
        self.pub = rospy.Publisher('joint_states', JointState)
        self._msg = JointState();
        for i in range(0,6):
            self._msg.name.append("joint"+str(i+1))
        self._msg.position.append(0.00)
        self._msg.position.append(-1.57)
        self._msg.position.append(1.22)
        self._msg.position.append(0.0)
        self._msg.position.append(0.03)
        self._msg.position.append(-0.84)
    
    def loop(self):
        hz = 10
        r = rospy.Rate(hz)
        while not rospy.is_shutdown():
            self._msg.header.stamp = rospy.Time.now()
            self.pub.publish(self._msg)
            
            r.sleep()
            
if __name__ == '__main__':
    try:
        rospy.init_node('arm_state_publisher')
        asp = ArmStatePublisher()
        asp.loop()
    except rospy.ROSInterruptException: pass
