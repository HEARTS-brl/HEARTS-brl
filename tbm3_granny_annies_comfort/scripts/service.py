#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, String
import std_srvs.srv
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped



def location_pose_2D(req):
	return std_srvs.srv.EmptyResponse()



def service():
	rospy.init_node('a_node')
	s = rospy.Service('/roah_rsbb/tablet/map', std_srvs.srv.Empty, location_pose_2D)
	rospy.spin()



service()