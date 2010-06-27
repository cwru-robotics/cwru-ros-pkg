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

import gc
gc.enable()

#Todo:
#EXPAND the location where we add walls in so that we dont go outside the walls
#Fix the gradient expander
#Remove Globals
#USE DEQUE




MAP_SIZE = 8000
#WALL_SIZE = 41
WALL_SIZE = 31
BOX = 161
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

def constrainValue(val):
	if val < 0:
		return 0
	if val >= MAP_SIZE:
		return MAP_SIZE
	return val



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
	goal.target_pose.pose.position.y = convertToOdom(goalcell.y)
	#Fix Theta
	quaternion = tf.transformations.quaternion_about_axis(0, (0,0,1))
	goal.target_pose.pose.orientation = Quaternion(*quaternion)
	client.send_goal(goal)

class Planner:
	def __init__(self):
		#INITALIZE STUFF
		#wallGrid = normalDistributionArray(WALL_SIZE,15,400)
		#wallGrid = normalDistributionArray(WALL_SIZE,15,100)
		#reduceGrid = normalDistributionArray(REDUCE_SIZE,10,10000)
		self.wallGrid = normalDistributionArray(WALL_SIZE,6,500)
		self.walllist=list()
		self.walls = zeros((MAP_SIZE,MAP_SIZE),int)
		#reduceGrid = normalDistributionArray(REDUCE_SIZE,30,4000) + normalDistributionArray(REDUCE_SIZE,3,120)
		self.reduceGrid = normalDistributionArray(REDUCE_SIZE,13,4000)
		self.wallCost = zeros((MAP_SIZE,MAP_SIZE),float)
		self.gradCost = zeros((MAP_SIZE,MAP_SIZE),float)
		self.reduceCost = zeros((MAP_SIZE,MAP_SIZE),float)
		self.FullCost = zeros((MAP_SIZE,MAP_SIZE),float)
		self.expandMask = zeros((MAP_SIZE,MAP_SIZE),int)
		self.gradList = deque()
		self.hasRobot = False
		#robot = Cell(3488,2000)
		#robot = Cell(3500,2600)

	def setRobot(self,robot):
		self.nextrobot = robot
		if self.hasRobot == False:
			self.hasRobot = True
			self.gradCost[robot.x,robot.y]=1
			self.gradList.append(robot)

	#def isEdge(x,y):
	#	neigbors = findNeigbors(x,y)
	#	if self.gradCost[x,y] == 0:
	#		for cell in neigbors:
	#			if self.gradCost[cell.x,cell.y] != 0:
	#				return True
	#	return False

	def isInnerEdge(self,x,y):
		neigbors = findNeigbors(x,y)
		if self.gradCost[x,y] != 0:
			for cell in neigbors:
				if self.gradCost[cell.x,cell.y] == 0:
					return True

	#def expandToEdge(celltoSet):
	#	global gradCost
	#	global walls
	#	neighbors = findNeigbors(celltoSet.x,celltoSet.y)
	#	low = maxint
	#	for cell in neighbors:
	#		if gradCost[cell.x,cell.y]<low:
	#			if gradCost[cell.x,cell.y] != 0:
	#				low = gradCost[cell.x,cell.y]
	#	gradCost[celltoSet.x,celltoSet.y] = low + 1

	def fullCost(self,x,y):
		#return gradCost[x,y] - wallCost[x,y] - reduceCost[x,y]
		return self.FullCost[x,y]

	def robotExpand(self,cell):
		cells = deque()
		neighbors = findNeigbors(cell.x,cell.y)
		currentVal = self.fullCost(cell.x,cell.y)
		appended = False
		for neighbor in neighbors:
			value = self.fullCost(neighbor.x,neighbor.y)
			if self.gradCost[neighbor.x,neighbor.y] != 0:
				if value > currentVal:
					if self.expandMask[neighbor.x,neighbor.y]==0:
						self.expandMask[neighbor.x,neighbor.y]=1
						if self.isInnerEdge(neighbor.x,neighbor.y) == True:
							cells.append(neighbor)
							appended = True
						cells.extend(self.robotExpand(neighbor))
		if appended == False:
			cells.append(cell)
		return cells



	def robotAccendFull(self):	
		#make fullCost
		Xmin = constrainValue(self.robot.x - BOX)
		Xmax = constrainValue(self.robot.x + BOX)
		Ymin = constrainValue(self.robot.y - BOX)
		Ymax = constrainValue(self.robot.y + BOX)
		self.FullCost[Xmin:Xmax,Ymin:Ymax] =  self.gradCost[Xmin:Xmax,Ymin:Ymax] - self.wallCost[Xmin:Xmax,Ymin:Ymax] - self.reduceCost[Xmin:Xmax,Ymin:Ymax]


		#Clear expand mask
		m = self.expandMask[Xmin:Xmax,Ymin:Ymax]
		self.expandMask[Xmin:Xmax,Ymin:Ymax] = zeros(m.shape,int)
	
		x = self.robot.x
		y = self.robot.y
		low  = -maxint

		#Sometimes we get stupid and stuck in a local max this might remove this
		neighbors = findNeigbors(x,y)
		current = self.fullCost(x,y)
		Xlow = x
		Ylow = y
		#for neighbor in neighbors:
		#	if self.fullCost(neighbor.x,neighbor.y) < current:
		#		current = self.fullCost(neighbor.x,neighbor.y)
		#		Xlow = neighbor.x
		#		Ylow = neighbor.y

		#startCell = Cell(Xlow,Ylow)
		startCell = Cell(x,y)

		possibleEndPoints = self.robotExpand(startCell)
		goal = Cell(0,0)
		print len(possibleEndPoints)

		for point in possibleEndPoints:
			if self.fullCost(point.x,point.y)>low:
				low = self.fullCost(point.x,point.y)
				goal = point
		if goal.x == 0 and goal.y == 0:
			return self.robot
		return goal

	def makeWallCost(self):
		Xmin = constrainValue(self.robot.x - BOX)
		Xmax = constrainValue(self.robot.x + BOX)
		Ymin = constrainValue(self.robot.y - BOX)
		Ymax = constrainValue(self.robot.y + BOX)
		m = self.wallCost[Xmin:Xmax,Ymin:Ymax]
		self.wallCost[Xmin:Xmax,Ymin:Ymax] = zeros(m.shape,int)
		wallMid = (WALL_SIZE-1)/2
		for wall in self.walllist:
			if wall.x>Xmin and wall.x < Xmax and wall.y >Ymin and wall.y < Ymax:
				CopyMaskSum(self.wallCost,self.wallGrid,wall.x,wall.y)
				self.walls[wall.x,wall.y] = 1
	def expandGradient(self,cell):
		neighbors = findNeigbors(cell.x, cell.y)
		for neighbor in neighbors:
			if self.walls[neighbor.x,neighbor.y] ==  0:
				if self.gradCost[neighbor.x,neighbor.y] == 0:
					self.gradCost[neighbor.x,neighbor.y] = self.gradCost[cell.x,cell.y] + 1
					self.gradList.appendleft(neighbor)

	def makeGradient(self):	
		Xmin = constrainValue(self.robot.x - BOX)
		Xmax = constrainValue(self.robot.x + BOX)
		Ymin = constrainValue(self.robot.y - BOX)
		Ymax = constrainValue(self.robot.y + BOX)
		outList = deque()
		while self.gradList:	
			cell = self.gradList.pop();
			if cell.x > Xmin and cell.x < Xmax and cell.y > Ymin and Ymax > cell.y:
				self.expandGradient(cell)	
			else:
				outList.append(cell)
				
		self.gradList=outList


	def runUpdatePlanner(self):
		if self.hasRobot == True and self.walllist:
			self.robot = self.nextrobot
			start = time.time()
			self.makeWallCost()
			print "Wall Cost " ,  time.time() - start
			wallTime = time.time() - start
	

			start = time.time()
			self.makeGradient()
			print "Gradient " ,  time.time() - start
			gradTime = time.time()-start
		
			CopyMaskSum(self.reduceCost,self.reduceGrid,self.robot.x,self.robot.y)

			start = time.time()
	
			goal = self.robotAccendFull()
			print "Accending " ,  time.time() - start
			accendTime = time.time()-start
			print "Total time = ",wallTime+gradTime+accendTime
			return goal



#Might need a mutex around the planner.walls value!!!!!!!
def setObstacles(walls,planner):
	cells = list()	
	for point in walls.cells:
		wallX = convertFromOdom(point.x)
		wallY = convertFromOdom(point.y)
		cells.append(Cell(wallX,wallY))
	planner.walllist=cells


def setRobotPose(Odom,planner):
	robot = Cell(0,0)
	robot.y = convertFromOdom(Odom.pose.pose.position.y)	
	robot.x = convertFromOdom(Odom.pose.pose.position.x)
	planner.setRobot(robot)

def main():
	planner = Planner()
	rospy.init_node('harlie_obstacle_planner')
	rospy.Subscriber("/odom", Odometry, setRobotPose,planner)
	rospy.Subscriber("/move_base/global_costmap/inflated_obstacles", GridCells, setObstacles,planner)		
	rate = rospy.client.Rate(.1)
	while not rospy.is_shutdown():
		print time.time()
		goal = planner.runUpdatePlanner()	
		if goal:
			sendGoal(goal)
			#Show some data
			FUL=120
			data=split(planner.FullCost,planner.robot.x-FUL,planner.robot.y-FUL,planner.robot.x+FUL,planner.robot.y+FUL)
			l = mlab.surf(data)
			#mlab.show()
		rate.sleep()
if __name__ == '__main__':
    rospy.init_node('harlie_obstacle_planner')
    main()
