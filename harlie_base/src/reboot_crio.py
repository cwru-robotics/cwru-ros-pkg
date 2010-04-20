#!/usr/bin/env python

import roslib
roslib.load_manifest('harlie_base')
import rospy

from std_srvs.srv import Empty
rospy.wait_for_service('reboot_crio')
reset = rospy.ServiceProxy("reboot_crio", Empty)
reset()
