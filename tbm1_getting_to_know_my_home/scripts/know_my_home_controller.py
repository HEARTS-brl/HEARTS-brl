#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

class Controller():
    def __init__(self):
        self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)

        #subscribers


