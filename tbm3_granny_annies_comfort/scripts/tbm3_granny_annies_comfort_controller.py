#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from roah_rsbb_comm_ros.msg import BenchmarkState
from roah_rsbb_comm_ros.srv import Percentage
import std_srvs.srv


class Controller():
	
	def __init__(self):
        #Publishers
		self.tts_pub   = rospy.Publisher("/hearts/tts", String, queue_size=10)
		self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)       


        #Subscribers
		rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
		rospy.Subscriber("roah_rsbb/benchmark/state", BenchmarkState, self.benchmark_state_callback)
    
        #Services
		self.prepare = rospy.ServiceProxy('/roah_rsbb/end_prepare', std_srvs.srv.Empty)
		self.execute = rospy.ServiceProxy('/roah_rsbb/end_execute', std_srvs.srv.Empty)


		self.user_location = None

        # Disable head manager
        #?head_mgr = NavigationCameraMgr()
        #?head_mgr.head_mgr_as("disable")

        # List of unwanted words
		self.rm_words = [
		'of ', # nb trailing space to avoid corrupting "off"
		'in ', # nb trailing space to avoid corrupting "blind"
		'the',
		'please',
		'my',
		'me'
		]

        # Dictionary for DEVICE actions
		self.device_dict = {
		"switch on left light bedroom"  : "self.on_LLB()",
		"switch off left light bedroom" : "self.off_LLB()",
		"switch on right light bedroom" : "self.on_RLB()",
		"switch off right light bedroom": "self.off_RLB()",
		"switch on both lights bedroom" : "self.on_BLB()" ,
		"switch off both lights bedroom": "self.off_BLB()",
		"open blinds"            : "self.open_B()" ,
		"close blinds"           : "self.close_B()",
		"leave blinds half open" : "self.half_B()"
		}


        #self.objects = ['coke','water','juice','apple','lemon','glass']
        #self.rooms = ['bedroom', 'living_room']
        #self.device = ['light', 'blinds']
        #self.states = ['on', 'off', 'half', 'open', 'close']
        #self.verb = ['switch']

	def hearCommand_callback(self,data):
		rospy.loginfo('Heard a command')

		speech = str(data)
		speech = speech.lower()
		print("speech -:"+speech)

		# remove unwanted words
		for rm_item in self.rm_words:
			print("rm word: "+rm_item)
			speech = speech.replace(rm_item,'')
	
		words = speech.split(' ')	
		print("words  ....>")
		print(words)
		# rebuild key with single spaces

		lookupkey = ''
		for ii in range(1,len(words)):
			if words[ii] != '':
				lookupkey = lookupkey + words[ii]+' '
		lookupkey = lookupkey.strip()

		rospy.loginfo("lookup key: "+lookupkey)


		# look for speech in"device" dict
		print("code2exec ; "+ lookupkey)
		code2exec = self.device_dict.get(lookupkey)
		# check that key was found
		if code2exec != None:
			print('code 2 exec : '+ code2exec)
			exec(code2exec)
		else:
			print('code 2 exec : not found')	
		exec("code2exec")
  		return

	# exec def's for DEVICE actions
	def on_LLB(self):
		# ON  Left Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_1/on', std_srvs.srv.Empty)
		run_service()
		return

	def off_LLB(self):
		# OFF Left Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_1/off', std_srvs.srv.Empty)
		run_service()
		return	

	def on_RLB(self):
		# ON   Right Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_2/on', std_srvs.srv.Empty)
		run_service()
		return

	def off_RLB(self):
		# OFF Right Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_2/off', std_srvs.srv.Empty)
		run_service()
		return

	def on_BLB(self):
		# ON  Both Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_3/on', std_srvs.srv.Empty)
		run_service()
		return

	def off_BLB(self):
		# OFF Both Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_3/off', std_srvs.srv.Empty)
		run_service()
		return
    
	def open_B(self):
		# OPEN Blinds
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/blinds/max', std_srvs.srv.Empty)
		run_service()
		return

	def close_B(self):
		# CLOSE Blinds
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/blinds/min', std_srvs.srv.Empty)
		run_service()
		return	

	def half_B(self):
		# HALF CLOSE Blinds as at 27 Dec2017 does not work - percentae type problem
		p = Percentage()
		p.data = 50
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/blinds/set',Percentage)
		run_service(p)
		return	

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


	def listen(self, data):
		rospy.loginfo("Listening for command")
		sub = rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)

        #Possible Commands: 
        # Switch  
        # Open
        # Close
        # Set
        # Get / Bring / Find
        # Go Home


		sub.unregister()


	def device_operationsself(self):
		pass



	def main(self):
		print ("*** in main")

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
	import sys
	print("\n***")
	for p in sys.path:
		print(p)
	rospy.init_node('annies_comfort', anonymous=True)
	rospy.loginfo("annies comfort controller has started")

	controller = Controller()
	#exec("controller.on_LLB()")
	# exec("controller.off_LLB()")
	# exec("controller.on_RLB()")
	# exec("controller.off_RLB()")	
	# exec("controller.on_BLB()")
	# exec("controller.off_BLB()")
	# exec("controller.open_B()")
	# exec("controller.close_B()")
	# #exec("controller.half_B()")

	controller.main()
	rospy.spin()

   
