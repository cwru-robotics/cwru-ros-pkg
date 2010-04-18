#!/usr/bin/env python

import roslib
roslib.load_manifest('harlie_base')
import rospy

from std_srvs.srv import Empty

reset = rospy.ServiceProxy("reboot_crio", Empty)
reset()
