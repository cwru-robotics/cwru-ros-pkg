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

	# start path to elevator
	p2 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 7.0274
	p.yRef = 13.4659
	p.tangentAng = 0.7121
	p.rho = 0.0
	p.length = 1.7752
	p.vDes = 0.5
	p.accel = 0.2
	p2.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 7.6093
	p.yRef = 15.5085
	p.tangentAng = 0.7121
	p.rho = 0.8576
	p.length = 1.8316
	p.vDes = 0.4
	p.accel = 0.2
	p2.segs.append(p)
	paths['elevator'] = p2

	# start go to bathroom
	p3 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 8.4844
	p.yRef = 16.2639
	p.tangentAng =  2.2829
	p.rho = 100.0
	p.length = 0.0157 
	p.vDes = 0.008
	p.accel = 0.02
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef = 8.4778
	p.yRef = 16.2715
	p.tangentAng = -2.2326
	p.rho = 0.0
	p.length = 4.3851
	p.vDes = 0.5
	p.accel = 0.1
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 5.2784
	p.yRef = 13.2053
	p.tangentAng = -2.2326
	p.rho =  -1.5628
	p.length = 1.0865
	p.vDes = 0.3
	p.accel = 0.2
	p3.segs.append(p)


	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef =  4.8243
	p.yRef = 12.7544
	p.tangentAng = 2.3526
	p.rho = 0.0
	p.length = 5.6957
	p.vDes = 0.5
	p.accel = 0.2
	p3.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = 0.8112
	p.yRef = 16.7962
	p.tangentAng = 2.3526
	p.rho = -100.0
	p.length = 0.0157
	p.vDes = 0.008
	p.accel = 0.02
	p3.segs.append(p)
	paths['bathroom'] = p3

	# start path from bathroom to vending machines
	p4 = Path()
	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef =  0.8042
	p.yRef = 16.8174
	p.tangentAng =  0.7818
	p.rho = 100.0
	p.length = 0.0157 
	p.vDes = 0.003
	p.accel = 0.01
	p4.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef =  0.8113
	p.yRef = 16.8244
	p.tangentAng = 2.3606
	p.rho = 0.0
	p.length = 4.7258
	p.vDes = 0.5
	p.accel = 0.1
	p4.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 2
	p.xRef = -2.0264
	p.yRef = 20.6746
	p.tangentAng = 2.3606
	p.rho =  -1.3571
	p.length = 1.1600
	p.vDes = 0.4
	p.accel = 0.1
	p4.segs.append(p)

	p = PathSegment()
	p.frame_id = "map"
	p.segType = 1
	p.xRef =  -2.5480
	p.yRef = 21.1951
	p.tangentAng =  0.7864
	p.rho = 0.0
	p.length = 2.7161
	p.vDes = 0.5
	p.accel = 0.1
	p4.segs.append(p)
	paths['vending'] = p4

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
