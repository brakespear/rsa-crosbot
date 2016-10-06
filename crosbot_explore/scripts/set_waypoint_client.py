#!/usr/bin/env python

import sys
import rospy
from crosbot_explore.srv import *
from nav_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

#startPose = None

#def extractPose(odometry):
#	startPose = odometry.pose.pose

def position_subscriber():
	rospy.Subscriber("/odom",Odometry,extractPose)
	rospy.spin()

def get_astar_path(x, y, z):
	print "Getting a* path"
	rospy.wait_for_service("/get_path")
	try:
		get_path = rospy.ServiceProxy("/get_path", GetPath)
		endPoint = Pose()

		endPoint.position.x = x
		endPoint.position.y = y
		endPoint.position.z = z

		endPoint.orientation.x = 0
		endPoint.orientation.y = 0
		endPoint.orientation.z = 0
		endPoint.orientation.w = 1
		
		startPose = Pose()
		startPose.position.x = 2
		startPose.position.y = 2
		startPose.position.z = 0

		startPose.orientation.x = 0
		startPose.orientation.y = 0
		startPose.orientation.z = 0
		startPose.orientation.w = 1
		
		print "Start point is",startPose.position.x,startPose.position.y,startPose.position.z
		print "End point is",x,y,z
		path = get_path(startPose,endPoint)
		return path

		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def follow_path(path):
	rospy.wait_for_service("/explore/follow_path")
	try:
		follow_astar_path = rospy.ServiceProxy("/explore/follow_path",FollowPath)
		follow_astar_path(path)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
	rospy.init_node('set_waypoint_client_py')
	#position_subscriber()
	if len(sys.argv) == 4:
		x = int(sys.argv[1])
		y = int(sys.argv[2])
		z = int(sys.argv[3])
	else:
		print usage()
		sys.exit(1)
	path = get_astar_path(x,y,z)
	print "The path is",path
	follow_path(path)
	


