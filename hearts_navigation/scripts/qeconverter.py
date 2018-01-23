#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
import tf
import numpy as np
"""
: {
        "x" : 1.83319866657,
        "y" : -8.77737236023,
        "theta" : 0.070586556
    },
"""



def callback(data):
	angles = tf.transformations.euler_from_quaternion(np.array([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w]),'sxyz')
	theta = angles[2]
	rospy.loginfo(angles)
	print(": {")
	print('        "x" : '+str(data.pose.position.x)+',')
	print('        "y" : '+str(data.pose.position.y)+',')
	print('        "theta" : '+str(theta))
	print("    },")



rospy.init_node('quconverter', anonymous=True)
rospy.loginfo("converter has started")
rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)
rospy.spin()

