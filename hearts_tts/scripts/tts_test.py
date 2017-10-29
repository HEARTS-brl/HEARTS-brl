#!/usr/bin/python
import rospy
from std_msgs.msg import String
from time import sleep

pub = rospy.Publisher("/hearts/tts", String, queue_size = 1)
rospy.init_node("tts_test", anonymous=True)
while not rospy.is_shutdown():
    sleep(2)
    pub.publish("test")
