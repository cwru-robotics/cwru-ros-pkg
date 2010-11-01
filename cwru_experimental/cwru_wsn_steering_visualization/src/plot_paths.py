#! /usr/bin/env python

import roslib
roslib.load_manifest("cwru_wsn_steering_visualization")
import rospy

import matplotlib.pyplot as plt

from cwru_wsn_steering_msgs.msg import DesiredState
from cwru_base.msg import Pose

class PathPlotter:
    def __init__(self):
        self.desired_xs = []
        self.desired_ys = []
        self.actual_xs = []
        self.actual_ys = []
        self.figure = plt.figure()

    def append_state(self, state):
        self.desired_xs.append(state.x)
        self.desired_ys.append(state.y)

    def append_pose(self, pose):
        self.actual_xs.append(pose.x)
        self.actual_ys.append(pose.y)

    def plot_paths(self):
        ax = self.figure.add_subplot(111)
        lines = ax.plot(self.desired_xs,self.desired_ys, 'b', self.actual_xs, self.actual_ys, 'r')
        for line in lines:
                ax.add_line(line)
        plt.legend(lines, ('Desired', 'Actual'))
        plt.grid()
        plt.show()

if __name__ == "__main__":
    plotter = PathPlotter()
    rospy.init_node("steering_path_plotter")
    rospy.Subscriber("idealState", DesiredState, plotter.append_state)
    rospy.Subscriber("flipped_pose", Pose, plotter.append_pose)
    rospy.sleep(120.)
    plotter.plot_paths()
    rospy.spin()
