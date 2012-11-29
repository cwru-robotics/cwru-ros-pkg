#!/usr/bin/python

import roslib;
import rospy
from sensor_msgs.msg import JointState

class WheelStatePublisher():
    def __init__(self):
        self.pub = rospy.Publisher('joint_states', JointState)
    
    def loop(self):
        hz = 10
        r = rospy.Rate(hz)
        
        while not rospy.is_shutdown():
            msg = JointState();
            msg.header.stamp = rospy.Time.now()
            
            msg.name.append('l_caster_wheel_joint')
            msg.position.append(0)
            msg.name.append('r_caster_wheel_joint')
            msg.position.append(0)
            msg.name.append('l_caster_joint')
            msg.position.append(0)
            msg.name.append('r_caster_joint')
            msg.position.append(0)
            msg.name.append('l_drive_wheel_joint')
            msg.position.append(0)
            msg.name.append('r_drive_wheel_joint')
            msg.position.append(0)
            
            self.pub.publish(msg)
            
            r.sleep()
            
if __name__ == '__main__':
    try:
        rospy.init_node('wheel_state_publisher')
        wsp = WheelStatePublisher()
        wsp.loop()
    except rospy.ROSInterruptException: pass
