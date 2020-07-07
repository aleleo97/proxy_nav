#! /usr/bin/env python
#from tf import euler_from_quaternion
import math
import rospy
import array
from nav_msgs.msg import Path, Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sympy import symbols, IndexedBase, Idx
import sympy as sp
import numpy as np
import time 
from sympy import *
from opt import ConvexOpt
from model import DiscretizeandLinearizeGeneric

# --- Globals ---- 
# Position
init = PoseStamped()		# Initial position
init.header.frame_id = '/map'
goal = PoseStamped()		# Goal position
goal.header.frame_id = '/map'
pos = PoseStamped()			# Current position

# Mapping
costmap = OccupancyGrid()	# Costmap, the inflated occupancy grid
mapInfo = MapMetaData()		# Useful information about the map (e.g. resolution, width, height)
occupancyThresh = 50		# Value to decide safe zones for the robot in the occupancy grid

# Planning
gScore = []					# The gScore set

# Utilities (e.g. flags, etc)
haveInitial = 0
haveGoal = 0
#-----define the tf functions ----
def euler_to_quaternion( yaw ,roll = 0, pitch = 0  ):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

    
# ---- Subscriber Callbacks ----
# Initial pose msg comes from /initialpose topic, which is of PoseStamped() type
def initCallback(msg):
	global init
	global haveInitial
	init.pose = msg.pose.pose
	#quaternion = (init.pose.orientation.x, 
	#			  init.pose.orientation.y,
	#			  init.pose.orientation.z, 
	#			  init.pose.orientation.w)
	#rospy.loginfo("Initial pose received: (%s, %s, %s)",
	#			  init.pose.position.x,
	#			  init.pose.position.y,
				  #tf.transformations.euler_from_quaternion(quaternion)[2]*180.0/math.pi)
	haveInitial += 1

# Odometry msgs come from /odom topic, which are of the Odometry() type
def odomCallback(msg):
	global pos
	pos.pose = msg.pose.pose
	
# Goal msg comes from /move_base_simple/goal topic, which is of the PoseStamped() type
def goalCallback(msg):
	global goal
	global haveGoal
	goal = msg
	#quaternion = (goal.pose.orientation.x, 
	#			  goal.pose.orientation.y,
	#			  goal.pose.orientation.z, 
	#			  goal.pose.orientation.w)
	#rospy.loginfo("Goal pose received: (%s, %s, %s)",
	#			  goal.pose.position.x,
	#			  goal.pose.position.y,
				  #tf.transformations.euler_from_quaternion(quaternion)[2]*180.0/math.pi)
	haveGoal += 1

# Costmap comes from /move_base_node/global_costmap/costmap topic, which is of the OccupancyGrid() type
def costmapCallback(msg):
	global costmap
	costmap = msg
	
# Map meta data comes from /map_metadata topic, which is of the MapMetaData() type
def mapInfoCallback(msg):
	global mapInfo
	mapInfo = msg

def planner():
    # Initialize node
	rospy.init_node('global_planner', anonymous=True)
	# Create publisher
	pathPub = rospy.Publisher('/path_proxy', Path, queue_size=1)
	
	# Create subscribers
	odomSub = rospy.Subscriber('odom', Odometry, odomCallback)
	initSub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, initCallback)
	goalSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, goalCallback)
	#cMapSub = rospy.Subscriber('move_base_node/global_costmap/costmap', OccupancyGrid, costmapCallback)
	infoSub = rospy.Subscriber('map_metadata', MapMetaData, mapInfoCallback)
	
	
	# Set rate
	r = rospy.Rate(10) # 10 Hz
	while not rospy.is_shutdown():
		rospy.loginfo("Waiting for initial and goal poses")
		while haveGoal == 0:
			pass  
        # Set rate
		path = Path()
		path = search()
		path.header.frame_id = "map"


        # Publish the path continuously
		pathPub.publish(path)	


def search():
    global init
    global goal
	#let's define the variables of the class (u inputs and x states)
    u = IndexedBase('u')
    n_in = symbols('n_in ', integer=True)
    u[n_in]
    #you can change the number of input but not the name
    n_in = Idx('n_in', 2)
    x = IndexedBase('x')
    n_states = symbols('n_states', integer=True)
    x[n_states]
    #You can change the number of states not the name
    n_states = Idx('n_states', 3)
    angle_init = quaternion_to_euler(init.pose.orientation.x,init.pose.orientation.y,init.pose.orientation.z,init.pose.orientation.w)
    # steady state conditions
    x_init = [init.pose.position.x,init.pose.position.y,angle_init[0]]
    u_ss = [1,1]
    # final time
    tf = 10 #(seconds)
    #resolution
    k = 1
    # number of time points
    n = tf * k + 1      #total points
    # time points
    dt = tf/n
    t = np.linspace(0,tf,n)

    #define the ode of the system
    Z = [(.16/2)*(u[0]+u[1])*sp.cos((3.14/180)*x[2]),(.16/2)*(u[0]+u[1])*sp.sin((3.14/180)*x[2]),(.16/.55)*(u[0]-u[1])]
    eq = DiscretizeandLinearizeGeneric(Z,np.zeros(x[n_states].shape[0]),np.ones(u[n_in].shape[0]),n)

    # define inputs over time 
    u1= np.ones(n) * u_ss[0]
    u2= np.ones(n) * u_ss[1]
    uw = np.array( [u1,u2])
    angle_goal = quaternion_to_euler(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w)
    #define the goal  position and the condition initial of the velocity
    x_fin = [goal.pose.position.x,goal.pose.position.y,angle_goal[0]]
    u_in = [0,0]
    #set the start time 
    start = time.time()
    #iteration to find the optimum result
    for i in range (6):
        #resolution discrete sistem
        x1,x2,x3 = eq.disc(uw,n,dt,x_init)
        Ad_list,Bd_list,Cd_list = eq.get_list()

        #call the Convex optimization class to resolve the problem 
        cvx = ConvexOpt(n,x_init,x_fin,u_in,Ad_list,Bd_list,Cd_list)
        #tell to optimize the power 
        opt_pow = True
        #tell to optimize the rapidity of convergence
        opt_conv = True
        xout,uout = cvx.CVXOPT(opt_pow,opt_conv)
        uw = uout

    done = time.time()
    passed = done - start
    #print("time passed to find a solution: " + passed)
    #plot the true trajectory calculated take into account the estimated u vector with cvx optimization
    x1,x2,x3 = eq.disc(uw,n,dt,x_init)
    path = Path()
    for i in range(0,n):
        position = PoseStamped()
        position.pose.position.x = x1[i]
        position.pose.position.y = x2[i]
        quat = euler_to_quaternion(x3[i]*3.14/180)
        position.pose.orientation.x = quat[0]
        position.pose.orientation.y = quat[1]
        position.pose.orientation.z = quat[2]
        position.pose.orientation.w = quat[3]
        position.header.frame_id = '/map'
        path.poses.append(position)
    return path

def poseToGrid(pose):
	# Converts from pose in meters to pose in grid units (helper function)
	grid_x = int((pose.pose.position.x - mapInfo.origin.position.x) / mapInfo.resolution)
	grid_y = int((pose.pose.position.y - mapInfo.origin.position.y) / mapInfo.resolution)
	pose.pose.position.x = grid_x
	pose.pose.position.y = grid_y
	return pose
	
def gridToPose(pose):
	# Converts from grid units to pose in meters (helper function)
	x = (pose.pose.position.x*mapInfo.resolution) + mapInfo.origin.position.x
	y = (pose.pose.position.y*mapInfo.resolution) + mapInfo.origin.position.y
	pose.pose.position.x = x
	pose.pose.position.y = y
	return pose


if __name__ == "__main__":
	try:
		planner()
	except rospy.ROSInterruptException:
		pass