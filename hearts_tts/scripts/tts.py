#!/usr/bin/python
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

def callback(data):
    print("saying " + data.data)
    soundhandle.say(data.data)

soundhandle = SoundClient()
rospy.init_node("tts",anonymous=True)
rospy.Subscriber("/hearts/tts", String, callback)
rospy.spin()
