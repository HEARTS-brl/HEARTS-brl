#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
from actionlib_msgs.msg import GoalStatusArray, GoalID
from std_msgs.msg import String
from std_msgs.msg import UInt8
#from roah_rsb_comm_ros import BenchmarkState


class Navigator():

    def __init__(self):

        self.isNavigating = False
        self.currentGoal = Pose2D()
        self.lastGoal = Pose2D()

        self.pubGoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.pubStatus = rospy.Publisher('/hearts/navigation/status', String, queue_size=1)
        self.pubPose = rospy.Publisher('/hearts/navigation/current', String, queue_size=1)

        self.previous_state = ""
        rospy.Subscriber('move_base/status', GoalStatusArray, self.StatusCallback)   # Get status of plan

        rospy.Subscriber('hearts/navigation/goal', Pose2D, self.goalCallback)
        rospy.Subscriber('hearts/navigation/stop', String, self.stopCallback)

    def goalCallback(self, data):

        rospy.loginfo("Navigator: goal Callback")

        # Make sure the robot isn't already trying to go somewhere
        if(self.isNavigating == True):
            rospy.loginfo("Navigator: currently navigating, can't continue at the moment.")
            self.pubStatus.publish("busy")

        else:
            rospy.loginfo("Navigator: sending new goal pose")

            # @TODO add some form of checking to data:

            self.currentGoal = data

            # Convert Pose2D to PoseStamped
            t = PoseStamped()
            t.header.frame_id = "/map"
            t.pose.position.x = data.x
            t.pose.position.y = data.y
            t.pose.orientation.w = data.theta

            self.pubGoal.publish(t)
            self.isNavigating = True

    def stopCallback(self, data):

        if(data.data):
            rospy.loginfo("Navigator: Stop Callback")

    def StatusCallback(self, data):
        length_status = len(data.status_list)

        if length_status > 0:
            status = data.status_list[length_status-1].status
            rospy.loginfo('status: ' + str(status))
            
            if status == 4 or status == 5 or status == 9:
                status_msg = 'Fail'
                self.isNavigating = False
            elif status==3:
                status_msg = 'Success'
                self.lastGoal = self.currentGoal
                self.currentGoal = Pose2D()
                self.isNavigating = False
            else:
                status_msg = 'Active'
                self.isNavigating = True

            if self.previous_state != status_msg: 
                self.pubStatus.publish(status_msg)
                self.previous_state = status_msg

if __name__ == '__main__':
	rospy.init_node('Navigator', anonymous=True)
	rospy.loginfo("Navigator has started")
	n = Navigator()
	rospy.spin()
