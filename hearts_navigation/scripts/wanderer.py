#!/usr/bin/env python

# "Not all those who wander are lost"
#                           -- What you Tolkien 'bout

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
import json

# pose = Pose value
# location = string value

class Wander():
    def __init__(self):

        self.nodeName = "wander"
        self.running = False

        rospy.Subscriber("hearts/controller/routine", String, self.goCallback)

        # @TODO publish to own cmd_vel_mux msg.
        self.pubVel = rospy.Publisher('hearts/input/teleop', Twist, queue_size=10)
        rospy.Subscriber("hearts/navigation/stop", String, self.haltCallback)

        # These limit the speed of the robot / weighted
        self.linearSpeed = 0.2
        self.angularSpeed = 0.2

        self.halt = False

        self.currentTwist = Twist()

        self.loop()

    def goCallback(self, data):
        if(data.data == self.nodeName):
            self.running = True
        else:
            self.running = False

    # halfCallback sets self.halt value to current half from avoid
    def haltCallback(self, data):

        if(data.data == "True"):
            self.halt = True
        else:
            self.halt = False

    # Check the robot is allowed to be where it is (this is useful in simulation,
    #  at least.) Fake a halt msg if it's out of bounds.
    def checkWorldLimits(self):
        pass

    # loop
    def loop(self):

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            # Clear temp twist msg every time.
            t = Twist()
            # Move forward if there is nothing detected, turn if there is:
            if (self.halt == False):
                print "Forward"
                t.linear.x = 1.0
                # @TODO add inverse releationship between object distance & speed
                # val = self.halt.value - 0.5 # remove the threshold val
                # if val >= 1.0: val = 1.0 # set max limit
                # if val == 0.0: val = 0.001
                # t.linear.x = t.linear.x * val
                t.linear.x = t.linear.x * self.linearSpeed
            else:
                print "Turn"
                t.angular.z = 1.0 * self.angularSpeed

            # send twist msg
            self.pubVel.publish(t)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("wander_mode", anonymous=True)
    wander = Wander()
    rospy.spin()
