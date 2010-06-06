#!/usr/bin/env python
import roslib
roslib.load_manifest('harlie_goal_planner')
import rospy

import tf
import actionlib

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells

from numpy import *
from enthought.mayavi import mlab
from sys import maxint
from collections import deque
import time
#from scipy import weave 


#import gc
#gc.disable()

#Todo:
#EXPAND the location where we add walls in so that we dont go outside the walls
#Fix the gradient expander
#Remove Globals
#USE DEQUE

MAP_SIZE = 4000
#WALL_SIZE = 41
WALL_SIZE = 31
BOX = 61
REDUCE_SIZE=81
EXTRA = 100


#Defines x y and cost
class Cell:
	x = 0
	y = 0
	cost = 0
	def __init__(self,X,Y):
		self.x=X
		self.y=Y
def isOdd(num):
	return num & 1 and True or False
def isValidXY(x,y):
	if x>=0 and x<MAP_SIZE:
		if y>=0 and y<MAP_SIZE:
			return True
	return False
def findNeigbors(x,y):
	neigbors = list()
	if isValidXY(x+1,y):
		neigbors.append(Cell(x+1,y))
	if isValidXY(x-1,y):
		neigbors.append(Cell(x-1,y))
	if isValidXY(x,y+1):
		neigbors.append(Cell(x,y+1))
	if isValidXY(x,y-1):
		neigbors.append(Cell(x,y-1))
	if isValidXY(x+1,y-1):
		neigbors.append(Cell(x+1,y-1))
	if isValidXY(x-1,y+1):
		neigbors.append(Cell(x-1,y+1))
	if isValidXY(x+1,y+1):
		neigbors.append(Cell(x+1,y+1))
	if isValidXY(x-1,y-1):
		neigbors.append(Cell(x-1,y-1,))
	return neigbors
def normalDistributionArray(Size,row,factor):
	dist = ones((Size,Size))
	if isOdd(Size) == False:
		print "normalDistributionArray was sent an even number. This will not work"
	mid = (Size-1)/2
	for X in xrange(Size):
		for Y in xrange(Size):
			x=X-mid
			y=Y-mid
			location=sqrt(x*x+y*y)
			dist[X][Y] = (1/sqrt(2*row*row*3.14))*exp(-1*((location*location)/(2*row*row))) * factor
	return dist
def isEdge(x,y):
	neigbors = findNeigbors(x,y)
	if gradCost[x,y] == 0:
		for cell in neigbors:
			if gradCost[cell.x,cell.y] != 0:
				return True
	return False
def isInnerEdge(x,y):
	neigbors = findNeigbors(x,y)
	if gradCost[x,y] != 0:
		for cell in neigbors:
			if gradCost[cell.x,cell.y] == 0:
				return True
def constrainValue(val):
	if val < 0:
		return 0
	if val >= MAP_SIZE:
		return MAP_SIZE
	return val

def fullCost(x,y):
	global FullCost
	#return gradCost[x,y] - wallCost[x,y] - reduceCost[x,y]
	return FullCost[x,y]

def robotExpand(cell,expandMask,gradCost):
	cells = deque()
	neighbors = findNeigbors(cell.x,cell.y)
	currentVal = fullCost(cell.x,cell.y)
	appended = False
	for neighbor in neighbors:
		value = fullCost(neighbor.x,neighbor.y)
		if gradCost[neighbor.x,neighbor.y] != 0:
			if value > currentVal:
				if expandMask[neighbor.x,neighbor.y]==0:
					expandMask[neighbor.x,neighbor.y]=1
					if isInnerEdge(neighbor.x,neighbor.y) == True:
						cells.append(neighbor)
						appended = True
					cells.extend(robotExpand(neighbor,expandMask,gradCost))
	if appended == False:
		cells.append(cell)
	return cells



def robotAccendFull(expandMask,robot,gradCost):		
	#Clear expand mask
	Xmin = constrainValue(robot.x - BOX)
	Xmax = constrainValue(robot.x + BOX)
	Ymin = constrainValue(robot.y - BOX)
	Ymax = constrainValue(robot.y + BOX)
	m = expandMask[Xmin:Xmax,Ymin:Ymax]
	m = zeros(m.shape,int)
	
	x = robot.x
	y = robot.y
	notEdge = True
	low  = -maxint

	#Sometimes we get stupid and stuck in a local max this might remove this
	#neighbors = findNeigbors(x,y)
	#current = fullCost(x,y);
	#Xlow = x
	#Ylow = y
	#for neighbor in neighbors:
	#	if fullCost(neighbor.x,neighbor.y) < current:
	#		current = fullCost(neighbor.x,neighbor.y)
	#		Xlow = neighbor.x
	#		Ylow = neighbor.y

	#startCell = Cell(Xlow,Ylow)
	startCell = Cell(x,y)

	possibleEndPoints = robotExpand(startCell,expandMask,gradCost)
	goal = Cell(0,0)
	print len(possibleEndPoints)

	for point in possibleEndPoints:
		if fullCost(point.x,point.y)>low:
			#print "Cost = ",fullCost(point.x,point.y)
			low = fullCost(point.x,point.y)
			goal = point
	if goal.x == 0 and goal.y == 0:
		return robot
	return goal

def split(a,xMin,yMin,xMax,yMax):
	b=vsplit(a,[xMax])[0]
	c=vsplit(b,[xMin])[1]
	d=hsplit(c,[yMax])[0]
	e=hsplit(d,[yMin])[1]
	return e

def CopyMaskSum (large, mask,x,y):
	xSize,ynon = mask.shape
	mid=(xSize-1)/2
	Xcorner = x - mid-1
	Ycorner = y - mid-1
	Xopp = mid + x
	Yopp = mid + y

	large[Xcorner:Xopp,Ycorner:Yopp] = large[Xcorner:Xopp,Ycorner:Yopp]  + mask[:,:] 


def expandGradient(cell,walls,gradCost,gradList):
	neighbors = findNeigbors(cell.x, cell.y)
	for neighbor in neighbors:
		if walls[neighbor.x,neighbor.y] ==  0:
			if gradCost[neighbor.x,neighbor.y] == 0:
				gradCost[neighbor.x,neighbor.y] = gradCost[cell.x,cell.y] + 1
				gradList.appendleft(neighbor)

def makeGradient(gradList,robot,walls,gradCost):	
	Xmin = robot.x - BOX
	Xmax = robot.x + BOX
	Ymin = robot.y - BOX
	Ymax = robot.y + BOX
	outList = deque()
	while gradList:	
		cell = gradList.pop();
		if cell.x > Xmin and cell.x < Xmax and cell.y > Ymin and Ymax > cell.y:
			expandGradient(cell,walls,gradCost,gradList)	
		else:
			outList.append(cell)
				
	return outList


def convertFromOdom(value):
	resolution = .05
	return (value/resolution) - (value%resolution)/resolution + MAP_SIZE/2

def convertToOdom(value):
 	resolution = .05
	return (value-MAP_SIZE/2)*resolution

def sendGoal(goalcell):
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = '/odom'
	goal.target_pose.header.stamp = rospy.Time.now()
	print "MOVING TO ", 	 convertToOdom(goalcell.x), convertToOdom(goalcell.y)
	goal.target_pose.pose.position.x = convertToOdom(goalcell.x)
	goal.target_pose.pose.position.y = -1*convertToOdom(goalcell.y)
	#Fix Theta
	quaternion = tf.transformations.quaternion_about_axis(0, (0,0,1))
	goal.target_pose.pose.orientation = Quaternion(*quaternion)

	client.send_goal(goal)
	#Do we wait for a resulut
	#client.wait_for_result()
	#if client.get_state() == GoalStatus.SUCCEEDED:
    	#	rospy.loginfo("Goal executed successfully")
    	#else:
    	#   	rospy.logerr("Could not execute goal for some reason")


global gradList
global robot
global gradCost
global reduceCost
global initalized
global FullCost
initalized = False
hasOdom = False
robot = Cell(23,23)
#INITALIZE STUFF
wallGrid = normalDistributionArray(WALL_SIZE,6,500)
#reduceGrid = normalDistributionArray(REDUCE_SIZE,30,4000) + normalDistributionArray(REDUCE_SIZE,3,120)
reduceGrid = normalDistributionArray(REDUCE_SIZE,15,4000)
wallCost = zeros((MAP_SIZE,MAP_SIZE),float)
isWall = zeros((MAP_SIZE,MAP_SIZE),int)
gradCost = zeros((MAP_SIZE,MAP_SIZE),float)
reduceCost = zeros((MAP_SIZE,MAP_SIZE),float)
FullCost = zeros((MAP_SIZE,MAP_SIZE),float)
expandMask = zeros((MAP_SIZE,MAP_SIZE),int)
gradList = deque()
#robot = Cell(3488,2000)
#robot = Cell(3500,2600)
robot = Cell(3500,2400)
gradCost[robot.x,robot.y]=1
gradList.append(robot)

def setObstacles(walls):
	global robot
	global gradList
	global initalized
	global gradCost
	global reduceCost
	global FullCost
	if hasOdom == True:
		initalized = True
	if initalized != False:
			
		print "Robot x = ", robot.x , " y = ", robot.y
		start = time.time()
		Xmin = constrainValue(robot.x - BOX-WALL_SIZE/2)
		Xmax = constrainValue(robot.x + BOX+WALL_SIZE/2)
		Ymin = constrainValue(robot.y - BOX-WALL_SIZE/2)
		Ymax = constrainValue(robot.y + BOX+WALL_SIZE/2)	
		m = wallCost[Xmin:Xmax,Ymin:Ymax]
		m = zeros(m.shape,float)
		isWall[Xmin:Xmax,Ymin:Ymax]=zeros(m.shape,int)
		for point in walls.cells:
			wallX = convertFromOdom(point.x)
			wallY = convertFromOdom(point.y)
			if wallX < Xmax and wallX > Xmin and wallY>Ymin and wallY<Ymax:
				CopyMaskSum(wallCost,wallGrid,wallX,wallY)
				isWall[wallX,wallY]=1

	
		print "Wall Cost " ,  time.time() - start
		wallTime = time.time() - start

		start = time.time()
		gradList = makeGradient(gradList,robot,isWall,gradCost)
		print "Gradient " ,  time.time() - start
		gradTime = time.time()-start
		FUL = BOX+EXTRA
		#l = mlab.surf(split(gradCost,robot.x-BOX,robot.y-BOX,robot.x+BOX,robot.y+BOX))

		CopyMaskSum(reduceCost,reduceGrid,robot.x,robot.y)
		start = time.time()


		Xmin = robot.x - BOX
		Xmax = robot.x + BOX
		Ymin = robot.y - BOX
		Ymax = robot.y + BOX
		start = time.time()	
		FullCost[Xmin:Xmax,Ymin:Ymax] =  gradCost[Xmin:Xmax,Ymin:Ymax] - wallCost[Xmin:Xmax,Ymin:Ymax] - reduceCost[Xmin:Xmax,Ymin:Ymax]
		robot = robotAccendFull(expandMask,robot,gradCost)
		print "Accending " ,  time.time() - start
		accendTime = time.time()-start
		sendGoal(robot)
		print "Total time = ",wallTime+gradTime+accendTime

		print "Robot X,Y"
		print robot.x , robot.y
		data=split(FullCost,robot.x-FUL,robot.y-FUL,robot.x+FUL,robot.y+FUL)
		print data.shape
		l = mlab.surf(data)
		mlab.show()










def setRobotPose(Odom):
	global hasOdom
	robot.y = convertFromOdom(Odom.pose.pose.position.y)	
	robot.x = convertFromOdom(Odom.pose.pose.position.x)
	hasOdom=True


#def main():
#	rospy.Subscriber("/odom", Odometry, setRobotPose)
#	rospy.Subscriber("/move_base/global_costmap/inflated_obstacles", GridCells, setObstacles)		
#	rospy.spin()

rospy.init_node('harlie_obstacle_planner')
rospy.Subscriber("/odom", Odometry, setRobotPose)
rospy.Subscriber("/move_base/global_costmap/inflated_obstacles", GridCells, setObstacles)		
rospy.spin()



if __name__ == '__main__':
    rospy.init_node('harlie_obstacle_planner')
    #main(rospy.get_param('~filename'))
    main()






