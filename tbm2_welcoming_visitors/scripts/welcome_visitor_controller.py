#!/usr/bin/env python

# https://github.com/studioimaginaire/phue
# https://github.com/rockin-robot-challenge

import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Twist
from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState, DevicesState, TabletState
import std_srvs.srv

# activation: doorbell press detected
# notification:
# visitor: postman, doctor, deliman, unknown

class Controller():
    def __init__(self):
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        #self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        self.pub_location_goal = rospy.Publisher('/hearts/navigation/goal/location', String, queue_size=10)
        self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10) 
        self.detect_faces = False        

        # subscribers
        rospy.Subscriber("hearts/face/user",           String, self.face_callback)
        #rospy.Subscriber("hearts/voice/cmd",           String, self.voice_callback)
        rospy.Subscriber("/hearts/navigation/status", String, self.location_result_callback)
        #rospy.Subscriber("roah_devices/state",        DevicesState, self.state_callback)
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
        rospy.Subscriber("roah_rsbb/benchmark",        Benchmark, self.benchmark_callback)
        #rospy.Subscriber("roah_rsbb/benchmark",        String, self.benchmark_callback)
        rospy.Subscriber("roah_rsbb/devices/bell",     Empty, self.bell_callback)
        #rospy.Subscriber("hearts/front_door/leaving",     Empty, self.leaving_callback)
        
        self.tts_pub = rospy.Publisher("/hearts/tts", String, queue_size=10)
        self.location_result = ""
        #self.leaving = False

        #self.loop()

        self.has_scan_changed = False

    def scan_changed_callback(self, msg):
        self.has_scan_changed = (msg.data == "yes")

    def wait_for_scan_changed(self):
        self.has_scan_changed = False
        rospy.Subscriber("/scan_change", String, self.scan_changed_callback)
        
        while not self.has_scan_changed:
            rospy.sleep(1)
           
        # TODO: kill subscriber 
    
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
           
    #def voice_callback(self, data):
    #    rospy.loginfo("voice_callback: " + data.data)
    #    self.current_voice = data.data
 
    #def location_result_callback(self, data):
    #    rospy.loginfo("location_result_callback") 
  
#   def state_callback(self, data):
#        rospy.loginfo("state_callback")

    def benchmark_state_callback(self, data):
        if data.benchmark_state == BenchmarkState.STOP:
            rospy.loginfo("STOP")
        elif data.benchmark_state == BenchmarkState.PREPARE:
            rospy.loginfo("PREPARE")
            try:
                time.sleep(5)
                self.prepare()
            except:
                rospy.loginfo("Failed to reply PREPARE")
        elif data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE")

    def benchmark_callback(self, data):
        rospy.loginfo("benchmark_callback")
 
    #def leaving_callback(self, data):
    #    self.leaving = True

    def bell_callback(self, data):
        rospy.loginfo("bell_callback")
        
        self.say("I am coming")
        
        if self.move_to("hallway", 3) == False:
            self.say("I am unable to move to the front door")
            return
            
        self.say("please look towards the camera so that I can recognise you")
        
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
        self.tts_pub.publish(text)
        rospy.sleep(5)
        
    def move_to(self, location, count):
        rospy.loginfo("moving to \"" + location + "\" (" + str(count) + ")")
        msg = String()
        msg.data = location
        self.pub_location_goal.publish(msg)
        
        self.location_result = "Active"
        while self.location_result == "Active":
            rospy.sleep(1)
        
        if self.location_result != "Success" and count > 0:
            t = Twist()
            t.angular.z = 1.0
            self.pub_twist.publish(t)
            rospy.sleep(1)
            self.move_to(location, count - 1)
            
        return self.location_result == "Success"
        
    #def wait_until_left(self):
    #    while self.leaving == False:
    #        rospy.sleep(1)
    #    self.leaving = False

    def loop(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def location_result_callback(self, data):
        rospy.loginfo(data.data)
        self.location_result = data.data

    def process_face_postman(self):
                
        # 3.b speak to the postman, request him to open the door:  "I am here, please open the door"
        self.say("I will receive the post mail, please open the door")
 
        # 4. detect door is open
        rospy.sleep(3)

        # 5. speak to postman, instruct to follow robot: "Please follow me"
        self.say("Hello! Please follow me")

        # 6. move to kitchen
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the kitchen")
            return

        # 7. speak to postman, instruct to leave parcel on table: "Please leave the parcel on the table"
        self.say("Please leave the parcel on the table")

        # 8. wait
        # TODO
        rospy.sleep(5)

        # 9. speak to postman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 10. move to front door
        if self.move_to("entrance", 3) == False:
            self.say("I am unable to move to the front door")
            return

        # 11. bid postman farewell
        self.say("Thank you for visiting. Goodbye!")

        # 12. return to base
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the base")
            return

    def process_face_deliman(self):
        
        # 3. speak to the deliman, request him to open the door: "I am here, please open the door"
        self.say("I will receive the breakfast, please open the door")

        # 4. detect door is open
        rospy.sleep(3)

        # 5. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Hello! Please follow me")

        # 6. move to kitchen
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the kitchen")
            return

        # 7. speak to the deliman, instruct to leave breakfast box on the table: "Please leave the breakfast box on the table"
        self.say("Please leave the breakfast box on the table")
# topic: /y/compressed
        # 8. wait
        # TODO
        rospy.sleep(5)
        
        # 9. speak to the deliman, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 10. move to front door
        if self.move_to("entrance", 3) == False:
            self.say("I am unable to move to the front door")
            return

        # 11. bid deliman farewell
        self.say("Thank you for visiting. Goodbye!")

        # 12. return to base
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the base")
            return

    def process_face_doctor(self):
        # 1. speak to the doctor, "Hi Dr. Kimble, I am coming to open the door."
        self.say("Hello Dr. Kimble, please open the door")

        # 4. detect door is open
        rospy.sleep(3)

        # 5. speak to the doctor, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 6. move to bedroom
        if self.move_to("outside bedroom", 3) == False:
            self.say("I am unable to move to the bedroom")
            return

        # 7. speak to doctor, advise robot will wait
        self.say("I will wait here until you are done.")
    
        # 8. wait until doctor exits the bedroom
        rospy.sleep(5)
        self.wait_for_scan_changed()

        # 10. speak to the doctor, instruct to follow robot: "Please follow me"
        self.say("Please follow me")

        # 11. move to front door
        if self.move_to("entrance", 3) == False:
            self.say("I am unable to move to the front door")
            return

        # 12. bid farewell
        self.say("Thank you for visiting. Goodbye!")

        # 13. return to base
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the base")
            return

    def process_face_unrecognized(self):
        # 1. speak to visitor, "Sorry, I don't know you. I cannot open the door."
        self.say("Sorry, I don't know you. I cannot open the door")
        
        if self.move_to("kitchen", 3) == False:
            self.say("I am unable to move to the base")
            return

if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    rospy.loginfo("initialized controller node")
    controller = Controller()
    rospy.spin()
