#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from roah_rsbb_comm_ros.msg import BenchmarkState
import std_srvs.srv


class Controller():
    def __init__(self):
        #Publishers
        self.tts_pub = rospy.Publisher("/hearts/tts", String, queue_size=10)
        self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)       


        #Subscribers
        rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
    
        #Services
        self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
        self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)


        self.user_location = None

    
    ### When receiving a message from the "roah_rsbb/benchmark/state" topic, will then publish the corresponding state to "roah_rsbb/messages_save"
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
            self.main()
            
    def wait_for_call(self):
    	rospy.loginfo("Waiting for call")
    	self.wait = False
    	sub = rospy.Subscriber("/roah_rsbb/tablet/call", Empty, self.tablet_callback)
    	while self.wait == False:
    		rospy.sleep(5)
    	sub.unregister()

    def wait_for_user_location(self):
    	rospy.loginfo("Waiting for user location")
    	self.user_location = None
    	sub = rospy.Subscriber("/roah_rsbb/tablet/position", Pose2D, self.user_location_callback)
    	rospy.wait_for_service('/roah_rsbb/tablet/map')
    	user_location_service = rospy.ServiceProxy('/roah_rsbb/tablet/map', std_srvs.srv.Empty)
    	user_location_service()
    	rospy.loginfo("going to while loop")
    	while self.user_location is None:
    		rospy.sleep(5)
    	rospy.loginfo("Exiting while loop")
    	sub.unregister()


        # Callback functions
    def tablet_callback(self, msg):
    	self.wait = True

    def user_location_callback(self, msg):
    	rospy.loginfo("Waiting for user location callback")
    	rospy.loginfo(msg)
    	self.user_location = msg

    def navigation_callback(self, msg):
    	self.nav_status = msg.data


        ##Navigation Functions
    def move_to_pose2D(self, target_location_2D):
    	##publish granny annie's location
    	rospy.loginfo("Moving to Pose2D")
    	pub = rospy.Publisher('hearts/navigation/goal', Pose2D, queue_size = 10)
    	pub.publish(target_location_2D)

    	self.wait_to_arrive(5)
    	pub.unregister()

    def move_to_location(self, target_location):
    	rospy.loginfo("Moving to a location")
    	pub = rospy.Publisher('/hearts/navigation/goal/location', String, queue_size=10)
    	pub.publish(target_location)

    	self.wait_to_arrive(5)
    	pub.unregister()


    def wait_to_arrive(self, count): #when this function is called, must specify no of counts before it breaks out of infinite loop
    	rospy.loginfo("Checking Navigation Status")
    	sub = rospy.Subscriber("/hearts/navigation/status", String, self.navigation_callback)
    	self.nav_status = "Active"

    	while self.nav_status == "Active":
            rospy.sleep(1)

        sub.unregister()
        if self.nav_status == "Fail" and count > 0:
            t = Twist()
            t.angular.z = 1.0
            self.pub_twist.publish(t)
            rospy.sleep(1)
            self.wait_to_arrive(count - 1)
            
        return self.nav_status == "Success"


        ## Interactions
    def say(self, text):
        rospy.loginfo("saying \"" + text + "\"")
        rospy.sleep(1)
        self.tts_pub.publish(text)
        rospy.sleep(5)


    def main(self):

    	#wait for call 
    	self.wait_for_call()
    	#request location
    	self.wait_for_user_location()

    	#navigate to the user's location
    	self.move_to_pose2D(self.user_location)

    	#speak to granny annie
    	self.say("What can I do for you?")

    	#listen to granny annie

    	#reply to granny annie

        #check that command is correct

    	#execute command

    	#turn lights on/off

    	#do dimmer switch

    	#do blinds

    	#bring object

    	#go home


    	rospy.loginfo("End of programme")








if __name__ == '__main__':
	rospy.init_node('annies_comfort', anonymous=True)
	rospy.loginfo("annies comfort controller has started")
	controller = Controller()
	rospy.spin()

   
