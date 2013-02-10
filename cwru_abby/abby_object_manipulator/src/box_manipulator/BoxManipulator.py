#! /usr/env/bin python

from Queue import Queue
#TODO imports

class BoxManipulator:
    #TODO Get these from the parameter server
    plannerServiceName = "/ompl_planning/plan_kinematic_path"
    armGroupName = "irb_120"
    toolLinkName = "gripper_body"
    frameID = "/irb_120_base_link"
    gripperFingerLength = 0.35 #meters
    gripperCollisionName = "gripper"
    attachLinkName = "gripper_jaw_1"
    
    def __init__(self):
        self._tasks = Queue()
        self._moveArm = actionlib.SimpleActionClient('move_irb_120', MoveArmAction)
        self._pickServer = actionlib.SimpleActionServer('object_manipulation_pickup', PickupAction, auto_start=False)
        self._pickServer.register_goal_callback(self._pickGoalCB)
        self._pickServer.register_preempt_callback(self._pickPreemptCB)
        self._pickServer.start()
        self._placeServer = actionlib.SimpleActionServer('object_manipulation_place', PlaceAction, auto_start=False)
        self._placeServer.register_goal_callback(self._placeGoalCB)
        self._placeServer.register_preempt_callback(self._placePreemptCB)
        self._placeServer.start()
        self._gripperClient = rospy.ServiceProxy('abby_gripper/gripper', gripper)
        self._attachPub = rospy.Publisher('attached_collision_object', arm_navigation_msgs.AttachedCollisionObject)
        self._boundingBoxClient = rospy.ServiceProx('find_cluster_bounding_box', FindClusterBoundingBox)
    
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
        #Identify Vector
        #Vector is in the principal plane of the box most closely aligned with robot XZ. This plane will be tool YZ.
        #Vector is at a downward 30 degree angle
        #TODO VECTOR from bounding box fit goes here
        #Determine Orientation from vector and create constraint object
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = box.pose.header.frame_id
        o_constraint.link_name = self.toolLinkName
        o_constraint.absolute_roll_tolerance = 0.04
        o_constraint.absolute_pitch_tolerance = 0.04
        o_constraint.absolute_yaw_tolerance = 0.04
        #TODO orientation calculation goes here
        
        #Determine position and tolerance from vector and box size
        pos_constraint = PositionConstraint()
        pos_constraint.header = pos_constraint.header
        pos_constraint.link_name = pos_constraint.link_name
        #TODO position calculation goes here
        #Position of gripper finger tips is 2 cm back from edge of bounding box
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
        goal.planner_service_name = "/ompl_planning/plan_kinematic_path"
        
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
    
    def __init__(self, type=0, move_goal=arm_navigation_msgs.MoveArmActionGoal(), object_name=""):
        self.type = type
        self.move_goal = move_goal
        self.object_name = object_name
    