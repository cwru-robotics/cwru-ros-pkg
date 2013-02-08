#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Edward Vneator, Case Western Reserve University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Case Western Reserve University nor the names 
#    of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "esv@case.edu (Edward Venator)"

import roslib; roslib.load_manifest("cwru_utilities")
import rospy
from tabletop_object_detector.srv import TabletopSegmentation

if __name__ == '__main__':
    rospy.init_node('tabletop_object_detector_wrapper')
    rospy.loginfo('Waiting for tabletop_segmentation service')
    rospy.wait_for_service('tabletop_segmentation')
    tabletop_detection = rospy.ServiceProxy('tabletop_segmentation', TabletopSegmentation)
    rospy.loginfo('Connected to tabletop segmentation service')
    timer = rospy.Rate(.2)
    while not rospy.is_shutdown():
        resp = tabletop_detection()
        if resp.result == resp.SUCCESS:
            rospy.loginfo("Tabletop detection service returned %d clusters", len(resp.clusters))
        elif resp.result == resp.NO_TABLE:
            rospy.loginfo("No table detected")
        elif resp.result == resp.NO_CLOUD_RECEIVED:
            rospy.logwarn("No cloud received")
        elif resp.result == resp.OTHER_ERROR:
            rospy.logerror("Tabletop segmentation error")
        timer.sleep()
