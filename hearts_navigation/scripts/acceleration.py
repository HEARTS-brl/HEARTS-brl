#!/usr/bin/env python

# "Show me how you drive and I'll tell you who you are."
#                               --  Dom Toretto, Fast & Furious

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
import json

# pose = Pose value
# location = string value

class Acc():
    def __init__(self):

        self.halt = False

        # todo; define these in the launch file
        # for turtlebot, use cmd_vel_mux/input/teleop
        self.pubVel = rospy.Publisher('/key_vel', Twist, queue_size=10)

        self.accLinearStep = 0.055
        self.accAngularStep = 0.055

        self.currentTwist = Twist()

        rospy.Subscriber("/hearts/input/teleop", Twist, self.inputCallback)
        rospy.Subscriber("/hearts/navigation/stop", String, self.haltCallback)

    def haltCallback(self, data):

        if data.data == "False":
            self.halt = False
        else:
            self.halt = True

    def inputCallback(self, data):

        if data.linear.x <= self.currentTwist.linear.x:
            self.currentTwist.linear.x = self.currentTwist.linear.x - self.accLinearStep
        elif data.linear.x >= self.currentTwist.linear.x:
            self.currentTwist.linear.x = self.currentTwist.linear.x + self.accLinearStep
        else:
            self.currentTwist.linear.x = data.linear.x

        if data.angular.z <= self.currentTwist.angular.z:
            self.currentTwist.angular.z = self.currentTwist.angular.z - self.accAngularStep
        elif data.angular.z >= self.currentTwist.angular.z:
            self.currentTwist.angular.z = self.currentTwist.angular.z + self.accAngularStep
        else:
            self.currentTwist.angular.z = data.angular.z

        #if(self.halt == True):
        #    self.currentTwist.linear.x = 0
        #    self.currentTwist.angular.z = 0

        self.pubVel.publish(self.currentTwist)

if __name__ == '__main__':
    rospy.init_node("acceleration_controller", anonymous=True)
    acc = Acc()
    rospy.spin()
