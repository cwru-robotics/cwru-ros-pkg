#! /usr/bin/env python
import roslib
roslib.load_manifest('cwru_wsn_steering')
import rospy

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
	p.vDes = 0.1
	p.accel = 0.05
	p2.segs.append(p)
	paths['elevator'] = p2
	return paths

class PathSender:
	def __init__(self):
		self.paths = makeDummyPaths()
		self.path_pub = rospy.Publisher('desired_path', Path)

	def run(self):
		import pdb
		pdb.set_trace()

if __name__ == "__main__":
	rospy.init_node("path_sender")
	ps = PathSender()
	ps.run()
	rospy.spin()
