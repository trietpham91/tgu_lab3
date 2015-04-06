import rospy 
import tf
import roslib
import time as systime
import math

from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

# implementation of A* algorithm
def Astar(start, goal):
	closedlist = []
	openlist = [start]
	came_from = []

	g_score[start] = 0
	f_score[start] = g_score[start] + heuristic_cost_estimate(start, goal)

	while (openlist.itemsize != 0):
		current = node_lowest_f_score(f_score)
		if (current == goal):
			return reconstruct_path(came_from, goal)

		openlist.remove(current)
		closedlist.append(current)
		for neighbor in neighbor_nodes(current):
			if closedlist.count(neighbor) != 0 :
				continue
			temp_g_score = g_score[current] + dist_between(current, neighbor)
			
			if (openlist.count(neighbor) == 0) or (temp_g_score < g_score[neighbor]):
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = g_score[neighbor] + heuristic_cost_estimate(neighbor, goal)
				if openlist.count(neighbor) == 0 :
					openlist.append(neighbor)
	return failure

def heuristic_cost_estimate(start, goal):
	xDis = goal.position.x - start.position.x
	yDis = goal.position.y - start.position.y
	return math.sqrt(xDis**2 + yDis**2)

def node_lowest_f_score(f_score):
	for item in f_score:
		if (f_score.index(item) == 0):
			returnItem = item
			smallest_f_score = f_score[item]
		elif (f_score[item] < smallest_f_score):
				returnItem = item
				smallest_f_score = f_score[item]
	return returnItem

def reconstruct_path(came_from, goal):	
	total_path = [current]
	while (came_from.count(current) != 0):
		current = came_from[current]
		total_path.append(current)
	return total_path

def neighbor_nodes(current):
	x = current.position.x
	y = current.position.y

	pose2 = Pose()
	pose2.position.x = x
	pose2.position.y = y+0.2

	pose3 = Pose()
	pose3.position.x = x+0.2
	pose3.position.y = y+0.2

	pose4 = Pose()
	pose4.position.x = x+0.2
	pose4.position.y = y

	pose5 = Pose()
	pose5.position.x = x+0.2
	pose5.position.y = y-0.2

	pose6 = Pose()
	pose6.position.x = x
	pose6.position.y = y-0.2

	pose7 = Pose()
	pose7.position.x = x-0.2
	pose7.position.y = y-0.2

	pose8 = Pose()
	pose8.position.x = x-0.2
	pose8.position.y = y

	pose9 = Pose()
	pose9.position.x = x-0.2
	pose9.position.y = y+0.2

	neighborlist = [pose2,pose3,pose4,pose5,pose6,pose7,pose8,pose9]
	return neighborlist

def dist_between(current, neighbor):
	xDis = neighbor.position.x - current.position.x
	yDis = neighbor.position.y - current.position.y
	return math.sqrt(xDis**2 + yDis**2)

#Callback for turtlebot odometry messages
#param msg: Incoming message of type nav_msgs/Odometry
#returns: nothing
def odometryCallback(msg):
	px = data.pose.pose.position.x
	py = data.pose.pose.position.y
	quat = data.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)

	global x
	global y
	global theta

	x = px
	y = py
	theta = yaw

#Callback for turtlebot mapping messages
#param msg: Income message of type nav_msgs/OccupancyGrid
#returns: nothing
def mapCallback(msg):
	print msg

#Callback for PoseWithCovarianceStamped
#param msg: Income message of type geometry_msgs/PoseWithCovarianceStamped
#returns: nothing
def posecovarianceCallback(msg):
	global start
	start = msg.pose.pose

#Callback for PoseStamped
#param msg: Income message of type geometry_msgs/PoseStamped
#returns: nothing
def poseCallback(msg):
	global goal
	goal = msg.pose.pose

def mainFunction():
	#initialize ros nod
	rospy.init_node('lab3')
	
	#Publishers
	global teleop_pub
	teleop_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
	path_pub = rospy.Publisher('/nav_msgs/Path',Path)
	
	#Subscribers
	global odom_sub
	global map_sub
	global poseco_sub
	global pose_sub
	odom_sub = rospy.Subscriber('/odom', Odometry, odometryCallback, queue_size=10)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=10)
	poseco_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, posecovarianceCallback, queue_size=10)
	pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, poseCallback, queue_size=10)	
	costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', CostMap)

	print 'Lab3 node setup complete'

	global start
	global goal	

	Astar(start, goal)

	while not rospy.is_shutdown():
		pass

	print 'Lab 3 node exiting'

#Main Function
if __name__ == '__main__':
	mainFunction()
