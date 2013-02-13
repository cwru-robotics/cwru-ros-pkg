#! /usr/bin/python

import roslib; roslib.load_manifest('abby_object_manipulator')
from Queue import Queue
import rospy
import actionlib
from object_manipulation_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, Point
from object_manipulation_msgs.srv import *
from arm_navigation_msgs.msg import *
from abby_gripper.srv import *
import math
from math import sin, cos
import copy
import numpy
import tf

def multiplyTuple(tup1, tup2):
    '''Cross-multiply two tuples (useful for quaternion multiplication)'''
    num1 = numpy.asarray(tup1)
    num2 = numpy.asarray(tup2)
    return tuple(numpy.cross(num1,num2))


def quaternionFromRPY(roll, pitch, yaw):
    '''Given Tait-Bryan roll, pitch, and yaw angles, create a quaternion tuple 
    in (x, y, z, w) order'''
    w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
    x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
    y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
    z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
    return (w, x, y, z)

def quaternionFromEuler(phi, theta, psi):
    '''Given Euler ZYZ angles, create a quaternion tuple in (x, y, z, w) order'''
    w = cos(phi/2)*cos(theta/2)*cos(psi/2) - sin(phi/2)*cos(theta/2)*sin(psi/2)#sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
    x = cos(phi/2)*cos(psi/2)*sin(theta/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)#cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2)
    y = cos(phi/2)*sin(theta/2)*sin(psi/2) - sin(phi/2)*cos(psi/2)*sin(theta/2)#cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
    z = cos(phi/2)*cos(theta/2)*sin(psi/2) + cos(theta/2)*cos(psi/2)*sin(phi/2)#cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
    return (x, y, z, w)

class BoxManipulator:
    #TODO Get these from the parameter server
    plannerServiceName = "/ompl_planning/plan_kinematic_path"
    armGroupName = "irb_120"
    armActionName = 'move_irb_120'
    toolLinkName = "gripper_body"
    frameID = "/irb_120_base_link"
    preGraspDistance = .1 #meters
    gripperFingerLength = 0.115 #meters
    gripperOpenWidth = 0.065 #meters
    gripperClosedWidth = 0.046 #meters
    gripperCollisionName = "gripper"
    attachLinkName = "gripper_jaw_1"
    def __init__(self):
        self._tasks = Queue()
        self._moveArm = actionlib.SimpleActionClient(self.armActionName, MoveArmAction)
        self._moveArm.wait_for_server()
        rospy.loginfo('Connected to arm action server.')
        self._gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        rospy.loginfo('Connected to gripper service.')
        self._attachPub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        rospy.loginfo('Publishing collision attachments on /attached_collision_object')
        self._boundingBoxClient = rospy.ServiceProxy('find_cluster_bounding_box', FindClusterBoundingBox)
        rospy.loginfo('Connected to bounding box service.')
        self._tf_listener = tf.TransformListener()
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._pickServer = actionlib.SimpleActionServer('object_manipulation_pickup', PickupAction, auto_start=False)
        self._pickServer.register_goal_callback(self._pickGoalCB)
        self._pickServer.register_preempt_callback(self._pickPreemptCB)
        self._placeServer = actionlib.SimpleActionServer('object_manipulation_place', PlaceAction, auto_start=False)
        self._placeServer.register_goal_callback(self._placeGoalCB)
        self._placeServer.register_preempt_callback(self._placePreemptCB)
        self._pickServer.start()
        rospy.loginfo('Started pickup action server.')
        self._placeServer.start()
        rospy.loginfo('Started place action server.')
        rospy.loginfo('Box Manipulator Ready for Action Requests')
    
    def clearTasks(self):
        rospy.loginfo('Clearing task list.')
        with self._tasks.mutex:
            self._tasks.queue.clear()
    
    def _pickGoalCB(self):
        rospy.loginfo('Got a pick request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._pickServer.accept_new_goal()
        #Create goal message for pregrasp position and add it to the task queue
        preGraspGoal = self._makePreGrasp(goal)
        if preGraspGoal == False:
            return False;
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,preGraspGoal))
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Add final approach task to queue
        #self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE, self._makeGraspPath(preGraspGoal)))
        #Add close gripper task to queue
        #self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_CLOSE))
        #Add attach object task to queue
        #self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_ATTACH, object_name = goal.target.collision_name))
        self.runNextTask()
    
    def _pickPreemptCB(self):
        rospy.loginfo('Got a pick preempt request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._pickServer.set_preempted()
    
    def _placeGoalCB(self):
        rospy.loginfo('Got a place request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._placeServer.accept_new_goal()
        #Create goal message for pregrasp position and add it to the task queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,self._makePlace(goal)))
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Add detach object task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_DETACH, object_name = goal.collision_object_name))
        self.runNextTask()
    
    def _placePreemptCB(self):
        rospy.loginfo('Got a place preempt request')
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._placeServer.set_preempted()
    
    def _moveArmDoneCB(self, state, result):
        '''If the arm successfully moved to the position, move on to the next task.'''
        if result.error_code.val == result.error_code.SUCCESS:
            self._tasks.task_done()
            self.runNextTask()
        else:
            rospy.logerr("Arm motion failed! Error code:%d",result.error_code.val)
            self.clearTasks()
    
    def _moveArmActiveCB(self):
        pass
    
    def _moveArmFeedbackCB(self, feedback):
        pass
    
    def runNextTask(self):
        '''Process the next task on the queue and send it to the appropriate action server.
        Warning: This function is recursive for non-move tasks. A lot of non-move tasks in a row
        might eat up a lot of stack memory. I need to fix this, but I'm lazy.'''
        task = self._tasks.get(True)
        
        if task.type == task.TYPE_OPEN:
            rospy.loginfo('Opening the gripper')
            self._gripperClient(gripperRequest.OPEN)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_CLOSE:
            rospy.loginfo('Closing the gripper')
            self._gripperClient(gripperRequest.CLOSE)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_MOVE:
            rospy.loginfo('Waiting for arm action server')
            self._moveArm.wait_for_server()
            rospy.loginfo('Moving the arm')
            self._moveArm.send_goal(task.move_goal, self._moveArmDoneCB, self._moveArmActiveCB, self._moveArmFeedbackCB)
        elif task.type == task.TYPE_ATTACH:
            rospy.loginfo('Attaching an object')
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
            obj.object.id = task.object_name
            obj.link_name = self.attachLinkName
            obj.touch_links = self.touchLinks
            self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_DETACH:
            rospy.loginfo('Detaching an object')
            obj = AttachedCollisionObject()
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = task.object_name
            self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        else:
            rospy.logwarn('Skippping unrecognized task in queue.')
            pass
        
            
    def _makePreGrasp(self, pickupGoal):
        '''Given a PickupGoal message, fit a box to the point cloud in the target, identify an 
        approach vector, and designate a pregrasp position along that vector. Returns a MoveArmGoal
        message to move to the preGrasp position'''
        
        #Fit box
        box = self._boundingBoxClient(pickupGoal.target.cluster)
        #Check that the box width and/or length are within the gripper limits
        useX = not(box.box_dims.x > self.gripperOpenWidth or box.box_dims.x < self.gripperClosedWidth)
        useY = not(box.box_dims.y > self.gripperOpenWidth or box.box_dims.y < self.gripperClosedWidth)
        if useX:
            rospy.logdebug("Box width is within gripper parameters (%f m)",box.box_dims.x)
        if useY:
            rospy.logdebug("Box depth is within gripper parameters (%f m)",box.box_dims.y)
        if not(useX or useY):
            rospy.logerr("Could not pick up the box because its dimensions are too large or small for the gripper")
            return False
        #Do all geometry in robot's base_link frame
        boxPose = self._tf_listener.transformPose('/base_link', box.pose)
        #Turn box pose into a tf
        self._tf_broadcaster.sendTransform(
                (boxPose.pose.position.x,boxPose.pose.position.y,boxPose.pose.position.z), 
                (boxPose.pose.orientation.x,boxPose.pose.orientation.y,boxPose.pose.orientation.z,boxPose.pose.orientation.w),
                rospy.Time.now(),
                pickupGoal.collision_object_name,
                boxPose.header.frame_id)
        #Approach Vector is in the principal plane of the box most closely aligned with robot XZ. This plane will be tool YZ
        #Vector is at a downward 30 degree angle
        #Orientation of box should be (approximately) a pure z rotation from the robot base_link
        #Therefore the angle of rotation about the z axis is appoximately
        # 2 * acos(q_w)
        theta = 2 * math.acos(boxPose.pose.orientation.w)
        rospy.logdebug('Angle is %f',theta)
        #Determine what to rotate the pose quaternion by to get the vector quaternion
        #Start with a 30 degree downward rotation
        if useX and not useY:
            rospy.loginfo("Can only grab box along x axis")
            width = box.box_dims.x
            if theta >= math.pi/2 and theta <= 3*math.pi/2:
                conversionQuaternion = quaternionFromEuler(0, 5*math.pi/6, 0)
            else:
                conversionQuaternion = quaternionFromEuler(math.pi, 5*math.pi/6, 0)
        elif useY and not useX:
            rospy.loginfo("Can only grab box along y axis")
            width = box.box_dims.y
            if theta >= 0 and theta <= math.pi:
                conversionQuaternion = quaternionFromEuler(0, 7*math.pi/6, math.pi/2)
            else:
                conversionQuaternion = quaternionFromEuler(0, 7*math.pi/6, -math.pi/2)
        else:
            #Can use either face for pickup, so pick the one best aligned to the robot
            if theta >= math.pi/4 and theta >= 3*math.pi/4:
                rospy.loginfo("Grabbing box along -x axis")
                width = box.box_dims.y
                conversionQuaternion = quaternionFromEuler(math.pi, 5*math.pi/6, 0)
            elif theta >= 3*math.pi/4 and theta <= 5*math.pi/4:
                rospy.loginfo("Grabbing box along y axis")
                width = box.box_dims.x
                conversionQuaternion = quaternionFromEuler(-math.pi/2, 5*math.pi/6, 0)
            elif theta >= 5*math.pi/4 and theta <= 7*math.pi/4:
                rospy.loginfo("Grabbing box along x axis")
                width = box.box_dims.y
                conversionQuaternion = quaternionFromEuler(0, 5*math.pi/6, 0)
            else:
                rospy.loginfo("Grabbing box along -y axis")
                width = box.box_dims.x
                conversionQuaternion = quaternionFromEuler(math.pi/2, 5*math.pi/6, 0)
        print conversionQuaternion        
        #Box TF rotated into orientation of gripper
        self._tf_broadcaster.sendTransform(
                (0,0,0), 
                conversionQuaternion,
                rospy.Time.now(),
                pickupGoal.collision_object_name+'_rotated',
                pickupGoal.collision_object_name)
        #Pregrasp TF is rotated box TF translated back along the z axis
        distance = self.preGraspDistance + self.gripperFingerLength
        self._tf_broadcaster.sendTransform(
                (0,0,-distance),
                (0,0,0,1),
                rospy.Time.now(),
                pickupGoal.collision_object_name+'_pregrasp',
                pickupGoal.collision_object_name+'_rotated')
        (trans, rot) = self._tf_listener.lookupTransform(self.frameID, pickupGoal.collision_object_name+'_pregrasp',rospy.Time(0))
        #Determine Orientation from vector and create constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = self.frameID
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = Quaternion(rot[0],rot[1],rot[2],rot[3])
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        
        #Determine position and tolerance from vector and box size
        pos_constraint = PositionConstraint()
        pos_constraint.header = pos_constraint.header
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = Point(trans[0],trans[1],trans[2])
        #Position tolerance (in final tool frame) is:
        #  x = gripper open width - box x width
        #  y = object height * precision multiplier (<=1)
        #  z = 1 cm
        pos_constraint.constraint_region_shape.type = Shape.BOX
        pos_constraint.constraint_region_shape.dimensions = [
            self.gripperOpenWidth - width, 
            box.box_dims.z*0.2,
            0.01]
        
        preGraspGoal = MoveArmGoal()
        preGraspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        preGraspGoal.motion_plan_request = motion_plan_request
        return preGraspGoal
    
    def _makeGraspPath(self, preGraspGoal):
        '''Given a pregrasp MoveArmGoal message, generate a MoveArmGoal message to perform the final
        approach to the object to grasp it.'''
        
        graspGoal = MoveArmGoal()
        graspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        
        #Translate from pregrasp position to final position in a roughly straight line
        pos_constraint = copy.deepcopy(preGraspGoal.motion_plan_request.goal_constraints.position_constraints[0])
        distance = pos_constraint.position.z + self.preGraspDistance + self.gripperFingerLength
        pos_constraint.position.z = distance
        path_constraint = copy.deepcopy(preGraspGoal.motion_plan_request.goal_constraints.position_constraints[0])
        path_constraint.position.z = distance/2;
        path_constraint.constraint_region_shape.dimensions[2] += distance
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        motion_plan_request.path_constraints.position_constraints.append(path_constraint)
        #Orientation constraint is the same as for pregrasp
        motion_plan_request.goal_constraints.orientation_constraints = preGraspGoal.motion_plan_request.goal_constraints.orientation_constraints
        motion_plan_request.path_constraints.orientation_constraints = preGraspGoal.motion_plan_request.goal_constraints.orientation_constraints
        graspGoal.motion_plan_request = motion_plan_request
        
        #Turn off collision operations between the gripper and all objects
        collisionOperation = CollisionOperation(self.gripperCollisionName, 
                                    CollisionOperation.COLLISION_SET_ALL,
                                    0.0,
                                    CollisionOperation.DISABLE)
        graspGoal.operations.collision_operations.append(collisionOperation)
        
        return graspGoal
    
    def _makePlace(self, placeGoal):
        placePose = placeGoal.place_locations.pop()
        
        goal = MoveArmGoal()
        goal.planner_service_name = self.plannerServiceName
        
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = placePose.header.frame_id
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.position = placePose.pose.position
        pos_constraint.constraint_region_shape.type = Shape.BOX
        pos_constraint.constraint_region_shape.dimensions = [0.05, 0.05, 0.05]
        pos_constraint.constraint_region_orientation.x = 0;
        pos_constraint.constraint_region_orientation.y = 0;
        pos_constraint.constraint_region_orientation.z = 0;
        pos_constraint.constraint_region_orientation.w = 1.0;
        pos_constraint.weight = 1
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        
        o_constraint = OrientationConstraint()
        o_constraint.header = pos_constraint.header
        o_constraint.link_name = pos_constraint.link_name
        o_constraint.orientation = placePose.pose.orientation
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        o_constraint.weight = 1
        motion_plan_request.goal_constraints.orientation_constraints.append(o_constraint)
        
        goal.motion_plan_request = motion_plan_request
        return goal

class ManipulatorTask:
    
    TYPE_OPEN = 0
    TYPE_CLOSE = 1
    TYPE_MOVE = 2
    TYPE_ATTACH = 3
    TYPE_DETACH = 4
    
    def __init__(self, type=0, move_goal=MoveArmActionGoal(), object_name=""):
        self.type = type
        self.move_goal = move_goal
        self.object_name = object_name

if __name__ == "__main__":
    rospy.init_node('box_manipulator', log_level=rospy.DEBUG)
    rospy.loginfo('Box manipulator node started.')
    manipulator = BoxManipulator()
    rospy.spin()
