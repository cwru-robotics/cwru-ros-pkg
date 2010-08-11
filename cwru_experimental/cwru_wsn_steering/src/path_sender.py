#! /usr/bin/env python
import roslib
roslib.load_manifest('cwru_wsn_steering')
import rospy

from std_msgs.msg import String
from cwru_wsn_steering.msg import Path
from cwru_wsn_steering.msg import PathSegment


def makeDummyPaths():
	paths = {}
	p1 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 0.0436
	p.yRef = 2.18822
	p.tangentAng = 2.42426
	p.rho = 0.0
	p.length = 2.3202
	p.vDes = 0.5
	p.accel = 0.1
	p1.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = -1.6972
	p.yRef = 3.70692
	p.tangentAng = 2.42426
	p.rho = -100.0
	p.length = 0.0157
	p.vDes = 0.005
	p.accel = 0.05
	p1.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = -1.7532
	p.yRef = 3.847
	p.tangentAng = 0.8272
	p.rho = 0.0
	p.length = 13.0
	p.vDes = 0.5
	p.accel = 0.1
	p1.segs.append(p)
	paths['hallway'] = p1

	p2 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 7.0274
	p.yRef = 13.4659;
	p.tangentAng = 0.7121
	p.rho = 0.0
	p.length = 2.1659
	p.vDes = 0.5
	p.accel = 0.1
	p2.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 8.02
	p.yRef = 15.63
	p.tangentAng = 0.7121
	p.rho = 1.0
	p.length = 1.57
	p.vDes = 0.3
	p.accel = 0.05
	p2.segs.append(p)
	paths['elevator'] = p2

	p3 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 8.7
	p.yRef = 16.3
	p.tangentAng = 2.283
	p.rho = 100.0
	p.length = 0.0157 
	p.vDes = 0.003
	p.accel = 0.1
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 8.74
	p.yRef = 16.31
	p.tangentAng = -2.2740
	p.rho = 0.0
	p.length = 4.40
	p.vDes = 0.5
	p.accel = 0.1
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef =5.31
	p.yRef = 13.46
	p.tangentAng = -2.274
	p.rho = -1.30
	p.length = 1.21
	p.vDes = 0.4
	p.accel = 0.1
	p3.segs.append(p)


	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 4.82
	p.yRef = 12.75
	p.tangentAng = 2.35
	p.rho = 0.0
	p.length = 5.70
	p.vDes = 0.5
	p.accel = 0.1
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 0.81
	p.yRef = 16.80
	p.tangentAng = 2.350
	p.rho = -100.0
	p.length = 0.0157
	p.vDes = 0.004
	p.accel = 0.1
	p3.segs.append(p)
	paths['bathroom'] = p3
	
	return paths

class PathSender:
	def __init__(self):
		self.paths = makeDummyPaths()
		self.halt_path = Path()
		self.path_pub = rospy.Publisher('desired_path', Path)
		self.command_sub = rospy.Subscriber('chatter', String, self.handle_chatter)

	def handle_chatter(self, msg):
		if 'open' in msg.data or 'close' in msg.data:
			pass
		else:
		    path = self.paths.get(msg.data, self.halt_path)
		    self.path_pub.publish(path)

if __name__ == "__main__":
	rospy.init_node("path_sender")
	ps = PathSender()
	rospy.spin()
