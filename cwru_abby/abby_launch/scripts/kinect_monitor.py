#!/usr/bin/env python
'''This node borrowed from PR2 Object Manipulation Launch package by Matei 
    Ciocarlie, licenced under the terms of the BSD license.
    
    This node monitors the Kinect, and if it detects that the Kinect node has
    crashed, restarts the Kinect driver.'''

import roslib; roslib.load_manifest('sensor_msgs')

import subprocess
import threading
import traceback
import time

import rospy

from sensor_msgs.msg import CameraInfo

# Restart Kinect node if it stops publishing

class KinectMonitor(object):
    def __init__(self):
        self._lock = threading.RLock()

        self._last_cam_info   = time.time()
        self._last_spawn_time = time.time()

        self._kinect_node_name = rospy.get_param('~kinect_node_name', 'openni_node1')
        self._grace_period     = rospy.get_param('~grace_period',     30)

        self._sub = rospy.Subscriber('camera_info', CameraInfo, self._cb)

    def _cb(self, msg):
        with self._lock:
            self._last_cam_info = time.time()

    def _kill_kinect_node(self):
        rospy.logwarn('Trying to kill kinect node "%s". Node is stale' % self._kinect_node_name)
        
        retcode = -1
        try:
            retcode = subprocess.call('rosnode kill %s' % self._kinect_node_name, shell=True)
        except Exception, e:
            rospy.logerr('Unable to kill kinect node, caught exception:\n%s', traceback.format_exc())
            return

        if retcode != 0:
            rospy.logerr('Unable to kill kinect node, returned error code %d' % retcode)
            
    def update(self):
        with self._lock:
            if time.time() - self._last_cam_info > self._grace_period and time.time() - self._last_spawn_time > self._grace_period:
                self._kill_kinect_node()
                self._last_cam_info = time.time()

if __name__ == '__main__':
    rospy.init_node('kinect_monitor')

    monitor = KinectMonitor()
    
    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            monitor.update()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()
