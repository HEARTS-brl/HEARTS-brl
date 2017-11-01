#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyNode():

    def __init__(self):

        self.halt = False
        self.isJoy = False

        self.pubJoy = rospy.Publisher('hearts/input/teleop', Twist, queue_size=1)
        self.pubHalt = rospy.Publisher('hearts/stop', String, queue_size=10)
        self.pubVel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)

        rospy.Subscriber('joy', Joy, self.joystickCallback)

    def joystickCallback(self, data):

        t = Twist()

        t.linear.x = data.axes[1]   # Left Wiggle Stick - Vertical
        t.angular.z = data.axes[0]  # Left Wiggle Stick - Horizontal

        print data

        if(data.buttons[1] == 1):   # X
            if(self.halt == False):
                self.halt = True
                self.pubHalt.publish("True")
        else:
            if(self.halt == True):
                self.halt = False
                self.pubHalt.publish("False")

        if(data.buttons[5] == 1):   # R1
            self.pubVel.publish(t)
        else:
            self.pubJoy.publish(t)


if __name__ == '__main__':
	rospy.init_node('Joy', anonymous=True)
	rospy.loginfo("Joy has started")
	j = JoyNode()
	rospy.spin()
