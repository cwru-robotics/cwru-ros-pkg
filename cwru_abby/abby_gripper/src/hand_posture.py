#! /bin/python

import roslib; roslib.load_manifest('abby_gripper')
import rospy;
import actionlib;
import object_manipulation_msgs;

'''A hand posture controller for the 2-position parallel plate gripper on ABBY.
Implements the object_manipulation_msgs/GraspHandPostureExecutionAction action
server.
@author Edward Venator (esv@case.edu)
@todo Have this run on the arduino, replacing the service server currently
running there.
@license BSD
'''

class HandPostureController:
  # create messages that are used to publish feedback/result
  _feedback = object_manipulation_msgs.msg.GraspHandPostureExecutionActionFeedback()
  _result   = object_manipulation_msgs.msg.GraspHandPostureExecutionActionResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, object_manipulation_msgs.msg.GraspHandPostureExecutionAction, execute_cb=self.execute_cb)
    self._as.start()
    rospy.wait_for_service('abby_gripper/gripper')
    self.gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
    
  def execute_cb(self, goal):
    try:
	  if goal.goal.goal == goal.goal.PRE_GRASP:
        gripperGoal = gripperClient.OPEN
      elif goal.goal.goal == goal.goal.GRASP:
	    gripperGoal = gripperClient.OPENgripperClient.CLOSE
	  elif goal.goal.goal == goal.goal.release:
	    gripperGoal = gripperClient.OPEN
    try:
      resp = gripperClient(gripperGoal)
      if resp.success == resp.SUCCESS and resp.position == gripperGoal:
        self._result.result = self._result.result.SUCCESS
	    self._as.set_succeeded(self._result)
	  else:
	    rospy.logerror("Gripper service failed. Goal aborted.")
        self._result.result = self._result.result.FAILED
	    self._as.set_aborted(self._result)
    except rospy.ServiceException, e:
      rospy.logerror("Gripper service did not process request. Goal aborted.")
      self._result.result = self._result.result.ERROR
      self._as.set_aborted(self._result)
      
if __name__ == '__main__':
  rospy.init_node('hand_posture')
  HandPostureController(rospy.get_name())
  rospy.spin()