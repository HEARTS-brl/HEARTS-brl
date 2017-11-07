#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Twist

class Controller():
    def __init__(self):
        
        # Publishers - sends out goal locations, movement/turns, and speech
        self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        #self.pubGoal = rospy.Publisher('/hearts/navigation/goal', PoseStamped, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        
        self.pub_move = rospy.Publisher('/hearts/controller/move', Twist, queue_size = 10)

        # Subscribers - must listen for speech commands, location
        #rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
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

        self.loop()

    def loop(self):
            while not rospy.is_shutdown():
                t = Twist()
                ### insert intended values for movements here to fed to navigation here ###
                t.linear.x = 1.1
                t.linear.y = 2.2
                t.linear.z = 3.3
                t.angular.x = 1.1
                t.angular.y = 2.2
                t.angular.z = 3.3

                self.pub_move.publish(t)
                rospy.sleep(1)


          

        
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
        


if __name__ == '__main__':
    rospy.init_node('know_my_home', anonymous=True)
    rospy.loginfo("know my home controller has started")
    controller = Controller()
    rospy.spin()