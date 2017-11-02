#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

class Controller():
    def __init__(self):
        
        # Publishers - sends out goal locations, movement/turns, and speech
        
        self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        #self.pubGoal = rospy.Publisher('/hearts/navigation/goal', PoseStamped, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)

        # Subscribers - must listen for speech, location
        rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
        self.items = ['thing1','thing2','etc']
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": ["kitchen", "bedroom", "living", "dining", "family", "room"],
            "look": ["up", "down", "right", "left"],
            "see": self.items,
            "changed": self.items,
            "stop": []}
            
        
    def hearCommand_callback(self, data):
        s = data
        words = s.split(' ')
        global verb
        global subject
        possible_verbs = self.tbm1_commands_dict.keys()
        to_remove = ['to','the']
        #verb = ''
        #subject = ''
        for command in possible_verbs:
            if command in words:
                valid_command = True
                verb = command
                for r in to_remove:
                    if r in words:
                        words.remove(word)
                
                subject = " ".join(words)
            else:
                valid_command = False
        print('original',s)
        print('verb',verb)
        print('subject',subject)
        return 
        
