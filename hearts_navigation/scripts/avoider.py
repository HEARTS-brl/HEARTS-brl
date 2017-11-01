#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import json

# pose = Pose value
# location = string value

class Avoid():
    def __init__(self):

        # Publisher for object detected in front
        # currently publishing "True" / "False"
        # @TODO: publish the angles and/or distances
        self.pubHalt = rospy.Publisher('/hearts/navigation/stop', String, queue_size=10)
        # todo; define these in the launch file
        # for turtlebot, use cmd_vel_mux/input/teleop
        self.pubVel = rospy.Publisher('/key_vel', Twist, queue_size=10)
        # Distance threshold, something <= this will cause a msg to send
        self.avoidMinDistance = 0.0 # 0.5 meters
        self.avoidMaxDistance = 0.3 # 0.5 meters
        self.start = 0  #100
        self.end = 511  #411
        
        self.turnFlag = 0
        self.stopTime = 0
        self.stopTimeLimit = 100
        self.stopTurnLimit = 100
        # Reading the laser scan msg for object detection
        # for turtlebot, use laser_scan topic
        rospy.Subscriber("scan", LaserScan, self.scanCallback)

        self.haltFlag = False
        self.timer = 0

        self.loop()

    def loop(self):

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
        
            if(self.timer >= 2000):
                if(self.haltFlag or self.turnFlag != 0):
                    rospy.loginfo("stop: " + str(self.stopTime))
                    t = Twist()
                    t.linear.x = -0.1
                    

                    if self.stopTime >= self.stopTimeLimit:
                        rospy.loginfo("turn")
                        t.linear.x = 0.0
                        t.angular.z = 0.7
                        self.turnFlag = self.turnFlag + 1
                        
                        if(self.turnFlag >= self.stopTurnLimit):
                            rospy.loginfo("stop turn")
                            self.stopTime = 0
                            self.turnFlag = 0
                            self.timer = 0
                            
                    self.pubVel.publish(t)
                    self.stopTime = self.stopTime + 1
                    
                else:
                    self.stopTime = 0
            else:
                pass
                
            self.timer = self.timer + 1
            rate.sleep()

    def scanCallback(self, data):

        # Store scan info
        # notes: maybe doesn't have to do it (in the obj) every time a msg is send
        #  but this is probably less intensive than checking if they changed.
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        #self.angle_increment = data.angle_increment
        #self.time_increment = data.time_increment
        self.scan_time = data.scan_time
        self.range_min = data.range_min
        self.range_max = data.range_max
        # self.ranges = self.ranges


        # Bulk of the detection here:
        # tl;dr: check each laser angle reading if it's under the threshold.
        # also stores r1, r2 for min and max angles that something was detected
        # @TODO do something useful with this info, to provide a better behaviour
        i = 0 # counter
        r1 = -1 # first reading
        r2 = -1 # last reading

        for r in data.ranges:
        #for r in range(start, end):
            if i >= self.start and i <= self.end:
                if r >= self.avoidMinDistance and r <= self.avoidMaxDistance:
                    #print "Object at: " + str(i * data.angle_increment)
                    #print "\tReading: " + str(r)

                    if r1 == -1 and r2 == -1:
                        r1 = r
                    else:
                        r2 = r
            i = i + 1 # counter

        #print "\n\n"
        #print data.ranges[100]
        #print data.ranges[256]
        #print data.ranges[411]

        # If the r1 flag is set, something was detected, so send a msg
        if(r1 != -1):
            #print str(r1) + ", " + str(r2)
            #rospy.loginfo("stop")
            self.pubHalt.publish("True")
            self.haltFlag = True
        else:
            #rospy.loginfo("go")
            self.pubHalt.publish("False")
            self.haltFlag = False


if __name__ == '__main__':
    rospy.init_node("avoider_controller", anonymous=True)
    avoid = Avoid()
    rospy.spin()
