#!/usr/bin/env python

# https://github.com/studioimaginaire/phue
# https://github.com/rockin-robot-challenge

import rospy
import time
from std_msgs.msg import String
#from roah_devices.msg import DevicesState
from sound_play.libsoundplay import SoundClient
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
from std_msgs.msg import Empty
from roah_rsbb_comm_ros.msg import DevicesState
from geometry_msgs.msg import Pose2D
from roah_rsbb_comm_ros.msg import TabletState

# activation: doorbell press detected
# notification:
# visitor: postman, doctor, deliman, unknown

class Controller():
    def __init__(self):
        self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        self.pub_location_goal = rospy.Publisher('hearts/nav/location/goal', String, queue_size=10)
        self.detect_faces = False        
        self.soundhandle = SoundClient()

        # subscribers
        rospy.Subscriber("hearts/face/user",           String, self.face_callback)
        rospy.Subscriber("hearts/voice/cmd",           String, self.voice_callback)
        rospy.Subscriber("hearts/nav/location/result", String, self.location_result_callback)
        #rospy.Subscriber("roah_devices/state",        DevicesState, self.state_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber("roah_rsbb/benchmark",        Benchmark, self.benchmark_callback)
        #rospy.Subscriber("roah_rsbb/benchmark",        String, self.benchmark_callback)
        rospy.Subscriber("roah_rsbb/devices/bell",     Empty, self.bell_callback)
        rospy.Subscriber("hearts/front_door/leaving",     Empty, self.leaving_callback)

        self.location_result = ""
        self.leaving = False

        #self.loop()

    def face_callback(self, data):
        rospy.loginfo("face_callback: " + data.data)   
        if self.detect_faces == True: 
            if data.data == "postman":
                rospy.loginfo("detected postman")
                self.process_face_postman()
                self.detect_faces = False
            elif data.data == "deliman":
                rospy.loginfo("detected deliman")
                self.process_face_deliman()
                self.detect_faces = False
            elif data.data == "doctor":
                rospy.loginfo("detected doctor")
                self.process_face_doctor()
                self.detect_faces = False
            elif data.data == "unknown":
                rospy.loginfo("detected unrecognized person")
                self.process_face_unrecognized()             
                self.detect_faces = False  
           
    def voice_callback(self, data):
        rospy.loginfo("voice_callback: " + data.data)
        self.current_voice = data.data
 
    def location_result_callback(self, data):
        rospy.loginfo("location_result_callback") 
  
#   def state_callback(self, data):
#        rospy.loginfo("state_callback")

    def benchmark_state_callback(self, data):
         if data.benchmark_state == BenchmarkState.STOP:
             rospy.loginfo("benchmark_state_callback: STOP")
         elif data.benchmark_state == BenchmarkState.PREPARE:
             rospy.loginfo("benchmark_state_callback: PREPARE")
         elif data.benchmark_state == BenchmarkState.EXECUTE:
             rospy.loginfo("benchmark_state_callback: EXECUTE")

    def benchmark_callback(self, data):
        rospy.loginfo("benchmark_callback")
 
    def leaving_callback(self, data):
        self.leaving = True

    def bell_callback(self, data):
        rospy.loginfo("bell_callback")
        if self.detect_faces == False:
            # bell pressed, detect face
            self.detect_faces = True

    #def state_callback(self, data):
    #    rospy.loginfo("state_callback: " + data.data)
    #    if self.detect_faces == False and self.bell < data.bell:
    #        self.bell = data.bell
    #        # bell pressed, detect face
    #        self.detect_faces = True
    
    def say(self, text):
        rospy.loginfo("saying \"" + text + "\"")
        rospy.sleep(1)
        self.soundhandle.say(text)
        rospy.sleep(5)
        
    def move_to(self, location):
        rospy.loginfo("moving to \"" + location + "\"")
        msg = String()
        msg.data = location
        self.pub_location_goal.publish(msg)
        while self.location_result != "Success" and self.location_result != "Failure":
            rospy.sleep(1)
        
        return self.location_result == "Success"
        
    def wait_until_left(self):
        while self.leaving == False:
            rospy.sleep(1)
        self.leaving = False

    def loop(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def location_result_callback(self, data):
        rospy.loginfo(data.data)
        self.location_result = data.data

    def process_face_postman(self):
        # 1. speak to the postman: "Hello, I am coming to get the post mail." 
        self.say("Hello, I am coming to get the post mail")        

        # 2. move to front door        
	if self.move_to("front door") == False:
            self.say("I am unable to move to the front door")
            return
        
        # 3.b speak to the postman, request him to open the door:  "I am here, please open the door"
        self.say("I am here, please open the door")
 
        # 4. detect door is open
        rospy.sleep(5)

        # 5. speak to postman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 6. move to kitchen
        if self.move_to("kitchen") == False:
            self.say("I am unable to move to the kitchen")
            return

        # 7. speak to postman, instruct to leave parcel on table: "Please leave the parcel on the table"
        self.say("Please leave the parcel on the table")

        # 8. wait
        # TODO
        rospy.sleep(10)

        # 9. speak to postman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 10. move to front door
        if self.move_to("front door") == False:
            self.say("I am unable to move to the kitchen")
            return

        # 11. bid postman farewell
        self.say("Goodbye!")

        # 12. return to base
        if self.move_to("base") == False:
            self.say("I am unable to move to the base")
            return

    def process_face_deliman(self):
        # 1. speak to the deliman: "Hello, I am coming to get the breakfast." 
        self.say("Hello, I am coming to get the breakfast")
        
        # 2. move to front door
        if self.move_to("front door") == False:
            self.say("I am unable to move to the front door")
        
        # 3. speak to the deliman, request him to open the door: "I am here, please open the door"
        self.say("I am here, please open the door")

        # 4. detect door is open
        rospy.sleep(5)

        # 5. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 6. move to kitchen
        if self.move_to("kitchen") == False:
            self.say("I am unable to move to the kitchen")
            return

        # 7. speak to the deliman, instruct to leave breakfast box on the table: "Please leave the breakfast box on the table"
        self.say("Please leave the breakfast box on the table")
# topic: /y/compressed
        # 8. wait
        # TODO
        rospy.sleep(10)
        
        # 9. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 10. move to front door
        if self.move_to("front door") == False:
            self.say("I am unable to move to the front door")
            return

        # 11. bid deliman farewell
        self.say("Goodbye!")

        # 12. return to base
        if self.move_to("base") == False:
            self.say("I am unable to move to the base")
            return

    def process_face_doctor(self):
        # 1. speak to the doctor, "Hi Dr. Kimble, I am coming to open the door."
        self.say("Hi Dr. Kimble, I am coming to open the door")

        # 2. move to front door
        if self.move_to("front door") == False:
            self.say("I am unable to move to the front door")
            return

        # 3. speak to the doctor, request them to open the door: "I am here, please open the door"
        self.say("I am here, please open the door")

        # 4. detect door is open
        rospy.sleep(5)

        # 5. speak to the doctor, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 6. move to bedroom
        if self.move_to("bedroom") == False:
            self.say("I am unable to move to the bedroom")
            return

        # 7. speak to doctor, advise robot will wait
        self.say("I will wait here until you are done")
    
        # 8. wait until doctor exits the bedroom
        self.wait_until_left()

        # 9. move to the kitchen
        if self.move_to("kitchen") == False:
            self.say("I am unable to move to the kitchen")
            return

        # 10. speak to the doctor, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 11. move to front door
        if self.move_to("front door") == False:
            self.say("I am unable to move to the front door")
            return

        # 12. bid farewell
        self.say("Goodbye!")

        # 13. return to base
        if self.move_to("base") == False:
            self.say("I am unable to move to the base")
            return

    def process_face_unrecognized(self):
        # 1. speak to visitor, "Sorry, I don't know you. I cannot open the door."
        self.say("Sorry, I don't know you. I cannot open the door")
        return;

if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    rospy.loginfo("initialized controller node")
    controller = Controller()
    rospy.spin()
