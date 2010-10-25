#! /usr/bin/env python
import roslib
roslib.load_manifest('cwru_voice')
import rospy

from joy.msg import Joy
from cwru_voice_msgs.msg import StringStamped

class VerbalJoystick:
	def __init__(self):
		self.chat_sub = rospy.Subscriber('chatter', StringStamped, self.chatter_cb)
		self.joy_pub = rospy.Publisher('joy', Joy)
		self.joy_msg = Joy()
		self.joy_msg.axes.extend([0.0,0.0,1.0])

	def chatter_cb(self, msg):
		#Joy axis 0 is turning, 1 is % speed and 2 is throttle
		if 'right' in msg.data:
			self.joy_msg.axes[0] = self.joy_msg.axes[0] - 0.2
		elif 'left' in msg.data:
			self.joy_msg.axes[0] = self.joy_msg.axes[0] + 0.2
		elif 'forward' in msg.data:
			self.joy_msg.axes[0] = 0.0
			self.joy_msg.axes[1] = 0.4
		elif 'up' in msg.data:
			self.joy_msg.axes[1] = self.joy_msg.axes[1] + 0.2
		elif 'down' in msg.data:
			self.joy_msg.axes[1] = self.joy_msg.axes[1] - 0.2
		elif 'stop' in msg.data:
			self.joy_msg.axes[0] = 0.0
			self.joy_msg.axes[1] = 0.0
		else:
			pass
		self.joy_pub.publish(self.joy_msg)



if __name__ == '__main__':
	rospy.init_node('verbal_joystick')
	vbj = VerbalJoystick()
	rospy.spin()
