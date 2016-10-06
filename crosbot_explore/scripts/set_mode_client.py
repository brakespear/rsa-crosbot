#!/usr/bin/env python

import sys
import rospy
from crosbot_explore.srv import *



def set_mode(mode):
	rospy.wait_for_service("/explore/set_mode")
	try:
		set_mode = rospy.ServiceProxy("/explore/set_mode",SetMode)
		set_mode(mode)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
    return "%s [mode type]"%sys.argv[0]

if __name__ == "__main__":
	rospy.init_node('set_mode_client_py')
	if len(sys.argv) == 2:
		mode = int(sys.argv[1])
	else:
		print usage()
		sys.exit(1)
	print "Setting mode as",mode
	set_mode(mode)


