#! /usr/bin/python

import roslib; roslib.load_manifest('abby_object_manipulator')
from Queue import Queue
import rospy
import actionlib
from object_manipulation_msgs.msg import *
from geometry_msgs.msg import Pose
from object_manipulation_msgs.srv import *
from arm_navigation_msgs.msg import *
from abby_gripper.srv import *
import math

#TODO imports

class BoxManipulator:
    #TODO Get these from the parameter server
    plannerServiceName = "/ompl_planning/plan_kinematic_path"
    armGroupName = "irb_120"
    toolLinkName = "gripper_body"
    frameID = "/irb_120_base_link"
    preGraspDistance = .1 #meters
    gripperFingerLength = 0.115 #meters
    gripperCollisionName = "gripper"
    attachLinkName = "gripper_jaw_1"
    def __init__(self):
        self._tasks = Queue()
        self._moveArm = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
        rospy.loginfo('Connected to arm action server.')
        self._gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        rospy.loginfo('Connected to gripper service.')
        self._attachPub = rospy.Publisher('attached_collision_object', AttachedCollisionObject)
        rospy.loginfo('Publishing collision attachments on /attached_collision_object')
        self._boundingBoxClient = rospy.ServiceProxy('find_cluster_bounding_box', FindClusterBoundingBox)
        rospy.loginfo('Started place action server.')
        self._pickServer = actionlib.SimpleActionServer('object_manipulation_pickup', PickupAction, auto_start=False)
        self._pickServer.register_goal_callback(self._pickGoalCB)
        self._pickServer.register_preempt_callback(self._pickPreemptCB)
        self._pickServer.start()
        rospy.loginfo('Started pickup action server.')
        self._placeServer = actionlib.SimpleActionServer('object_manipulation_place', PlaceAction, auto_start=False)
        self._placeServer.register_goal_callback(self._placeGoalCB)
        self._placeServer.register_preempt_callback(self._placePreemptCB)
        self._placeServer.start()
        rospy.loginfo('Connected to bounding box service.')
        rospy.loginfo('Box Manipulator Ready for Action Requests')
    
    def clearTasks(self):
        with self._tasks.mutex:
            self._tasks.clear()
    
    def _pickGoalCB(self):
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._pickServer.acceptNewGoal().goal
        #Create goal message for pregrasp position and add it to the task queue
        preGraspGoal = self._makePreGrasp(goal)
        if preGraspGoal == False:
            return False;
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,preGraspGoal))
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Add final approach task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE, self._makeGraspPath(preGraspGoal)))
        #Add close gripper task to queue
        self._tasks.put_nowaitManipulatorTask((ManipulatorTask.TYPE_CLOSE))
        #Add attach object task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_ATTACH, object_name = goal.target.collision_name))
    
    def _pickPreemptCB(self):
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._pickServer.setPreempted()
    
    def _placeGoalCB(self):
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Accept the goal and read the object information
        goal = self._placeServer.acceptNewGoal().goal
        #Create goal message for pregrasp position and add it to the task queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_MOVE,self._makePlace))
        #Add open gripper task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_OPEN))
        #Add detach object task to queue
        self._tasks.put_nowait(ManipulatorTask(ManipulatorTask.TYPE_DETACH, object_name = goal.target.collision_name))
    
    def _placePreemptCB(self):
        #Clear tasks on the queue
        self.clearTasks()
        #Cancel current arm task
        self._moveArm.cancel_all_goals()
        #Set current goal preempted
        self._placeServer.setPreempted()
    
    def _moveArmDoneCB(self, state, result):
        '''If the arm successfully moved to the position, move on to the next task.'''
        if state == GoalStatus.SUCCEEDED:
            self._tasks.task_done()
            self.runNextTask()
    
    def _moveArmActiveCB(self, feedback):
        pass
    
    def _moveArmFeedbackCB(self):
        pass
    
    def runNextTask(self):
        '''Process the next task on the queue and send it to the appropriate action server.
        Warning: This function is recursive for non-move tasks. A lot of non-move tasks in a row
        might eat up a lot of stack memory. I need to fix this, but I'm lazy.'''
        task = self._tasks.get(True)
        
        if task.type == task.TYPE_OPEN:
            self._gripperClient(gripperRequest.OPEN)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == task.TYPE_CLOSED:
            self._gripperClient(gripperRequest.CLOSE)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == TYPE_MOVE:
            self._moveArm.sendGoal(task.move_goal, self._moveArmDoneCB, self._moveArmActiveCB, self._moveArmFeedbackCB)
        elif task.type == TYPE_ATTACH:
            obj = arm_navigation_msgs.AttachedCollisionObject
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.operation.operation = arm_navigation_msgs.CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
            obj.object.id = task.object_name
            obj.link_name = self.attachLinkName
            obj.touch_links = self.touchLinks
            self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        elif task.type == TYPE_DETACH:
            obj = arm_navigation_msgs.AttachedCollisionObject
            obj.object.header.stamp = rospy.get_rostime()
            obj.object.header.frame_id = self.frameID
            obj.operation.operation = arm_navigation_msgs.CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
            obj.object.id = task.object_name
            self._attachPub.publish(obj)
            self._tasks.task_done()
            self.runNextTask()
        else:
            pass
        
            
    def _makePreGrasp(self, pickupGoal):
        '''Given a PickupGoal message, fit a box to the point cloud in the target, identify an 
        approach vector, and designate a pregrasp position along that vector. Returns a MoveArmGoal
        message to move to the preGrasp position'''
        
        #Fit box
        box = self._boundingBoxClient(pickupGoal.target.cluster)
        #Check that the box width and/or length are within the gripper limits
        useX = (box.box_dims.x > self.gripperOpenWidth or box.box_dims.x < self.gripperClosedWidth)
        useY = (box.box_dims.y > self.gripperOpenWidth or box.box_dims.y < self.gripperClosedWidth)
        if not(useX or useY):
            rospy.logError("Could not pick up the box because its dimensions are too large or small for the gripper")
            return False
        #Identify Approach Vector, defined by a position (the box centroid) and an orientation
        approachVector = Pose()
        approachVector.position = box.pose.position
        #Vector is in the principal plane of the box most closely aligned with robot XZ. This plane will be tool YZ
        #Vector is at a downward 30 degree angle
        #Orientation of box should be (approximately) a pure z rotation from the robot base_link
        #Therefore the angle of rotation about the z axis is appoximately
        # 2 * acos(q_w)
        #TODO Calculate theta about z axis in base_link frame
        theta = math.pi/2
        #Determine what to rotate the pose quaternion by to get the vector quaternion
        conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, math.pi/6, 0)
        if useX and not useY:
            if theta >= 90 and theta<=270:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, math.pi) * conversionQuaternion
        elif useY and not useX:
            if theta >= 0 and theta<= 180:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, -math.pi/2) * conversionQuaternion
            else:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, math.pi/2) * conversionQuaternion
        else:
            #Can use either face for pickup, so pick the one best aligned to the robot
            if theta >= 45 and theta >= 135:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, -math.pi/2) * conversionQuaternion
            elif theta >= 135 and theta <= 225:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, math.pi) * conversionQuaternion
            elif theta >= 225 and theta <= 315:
                conversionQuaternion = createQuaternionMsgFromRollPitchYaw(0, 0, math.pi/2) * conversionQuaternion
        #Determine Orientation from vector and create constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = box.pose.header.frame_id
        o_constraint.link_name = self.toolLinkName
        o_constraint.orientation = box.pose.orientation * conversionQuaternion
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        
        #Determine position and tolerance from vector and box size
        pos_constraint = PositionConstraint()
        pos_constraint.header = pos_constraint.header
        pos_constraint.link_name = pos_constraint.link_name
        #Position of gripper finger tips is 10 cm back from edge of bounding box
        distance = self.preGraspDistance + self.gripperFingerLength
        pos_constraint.position = box.pose.position
        #TODO position calculation goes here
        #TODO position tolerance calculation goes here
        #Position tolerance (in final tool frame) is:
        #  x = gripper open width - box x width
        #  y = object height * precision multiplier (<=1)
        #  z = 1 cm
        
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
        preGraspGoal.goal.motion_plan_request = motion_plan_request
        return preGraspGoal
    
    def makeGraspPath(self, preGraspGoal):
        '''Given a pregrasp MoveArmGoal message, generate a MoveArmGoal message to perform the final
        approach to the object to grasp it.'''
        
        #Determine position from pregrasp position and orientation
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.frameID
        pos_constraint.link_name = self.toolLinkName
        pos_constraint.weight = 1
        #TODO position calculation goes here
        #Calculate path position constraint shape
        path_constraint = PositionConstraint()
        path_constraint.header = pos_constraint.header
        path_constraint.link_name = pos_constraint.link_name
        path_constraint.weight = 1
        #TODO path constraint goes here
        graspGoal = MoveArmGoal()
        graspGoal.planner_service_name = self.plannerServiceName
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.armGroupName
        motion_plan_request.num_planning_attempts = 1
        motion_plan_request.planner_id = ""
        motion_plan_request.allowed_planning_time = rospy.Duration(5,0)
        motion_plan_request.goal_constraints.position_constraints.append(pos_constraint)
        motion_plan_request.path_constraints.position_constraints.append(path_constraint)
        #Orientation constraint is the same as for pregrasp
        motion_plan_request.goal_constraints.orientation_constraints = preGraspGoal.motion_plan_request.goal_constraints.orientation_constraints
        motion_plan_request.path_constraints.orientation_constraints = preGraspGoal.motion_plan_request.goal_constraints.orientation_constraints
        graspGoal.goal.motion_plan_request = motion_plan_request
        
        #Turn off collision operations between the gripper and all objects
        collisionOperation = arm_navigation_msgs.CollisionOperation(self.gripperCollisionName, 
                                    arm_navigation_msgs.CollisionOperation.COLLISION_SET_ALL,
                                    arm_navigation_msgs.CollisionOperation.DISABLE)
        graspGoal.operations.collision_operations.append(collisionOperation)
        
        return graspGoal
    
    def _makePlace(self, placeGoal):
        placePose = placeGoal.place_locations.pop().pose
        
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
        pos_constraint.position = placePose.position
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
        o_constraint.orientation = placePose.orientation
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
    rospy.init_node('box_manipulator')
    rospy.loginfo('Box manipulator node started.')
    manipulator = BoxManipulator()
    rospy.spin()
