#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose 
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("arm_destination_test", anonymous=True)


##Publishers
p = rospy.Publisher('/arm_destination', Pose, queue_size = 10)

pose_target = Pose() 
q = quaternion_from_euler(-0.011, 1.57, 0.037) ## conversion from roll, pitch, and yaw into quaternions
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = 0.8
pose_target.position.y = -0.3
pose_target.position.z = 0.8

while not rospy.is_shutdown():
	p.publish(pose_target)
	rospy.loginfo("Publishing a location")
	rospy.loginfo(pose_target)
	rospy.sleep(0)
	rospy.spin()





