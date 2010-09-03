#!/usr/bin/env python
import roslib; roslib.load_manifest('cwru_voice')
import rospy
from std_msgs.msg import String
import festival
def callback(data):
	tts = festival.Festival()
	tts.say(data.data)
def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("speak", String, callback)
	rospy.spin()
if __name__ == '__main__':
	listener()

