#!/usr/bin/env python
##############################################################################################
# Author  : Derek Ripper/Joe Daly
# Created : Dec 2017
# Purpose : To fulfil ERL-SR competition Bench Mark TBM3 - Grannie Annies comfort
#           Written for the 22-26 Jan 2018 ERL competition in Edinburgh.       
#
##############################################################################################
# Updates : 
#
#
##############################################################################################
import rospy
import time
import std_srvs.srv
from   std_msgs.msg           import Empty, String
from   geometry_msgs.msg      import Pose2D, Pose, Twist, PoseStamped
from   roah_rsbb_comm_ros.msg import BenchmarkState
from   roah_rsbb_comm_ros.srv import Percentage



class Controller():
	
	def __init__(self):
		#Publishers  
		self.tts_pub   = rospy.Publisher("/hearts/tts", String, queue_size=10)
		self.pub_twist = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)       


		#Subscribers
		self.listen4cmd('on')
		self.listen4cmd('off')
		self.listen4ans('on')
		self.listen4ans('off')

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
		'a',
		'of',  
		'in',  
		'the',
		'please',
		'my',
		'to',
		'me'    # nb trailing space to avoid corrupting "home"
		]

		# List of words that ALL mean get
		self.get_words = [
		'bring',
		'find',
		'fetch'
		]

		# Dictionary for instructions to robot
		self.actions_dict = {
		"switch on left light bedroom"  : "self.on_LLB()",
		"switch off left light bedroom" : "self.off_LLB()",
		"switch on right light bedroom" : "self.on_RLB()",
		"switch off right light bedroom": "self.off_RLB()",
		"switch on both lights bedroom" : "self.on_BLB()" ,
		"switch off both lights bedroom": "self.off_BLB()",
        "set light dimmer half"         : "self.half_L()" ,
		"open blinds"                   : "self.open_B()" ,
		"close blinds"                  : "self.close_B()",
		"leave blinds half open"        : "self.half_B()" ,
		"go home"                       : "self.go_home()",
		"get cardboard box"             : "self.get('cardboard box')",
		"get coca-cola can"             : "self.get('coca-cola can')",
		"get coca cola can"             : "self.get('coca-cola can')",
		"get mug"                       : "self.get('mug')",
		"get candle"                    : "self.get('candle')",
		"get cup"                       : "self.get('cup')",
		"get reading glasses"           : "self.get('reading glasses')"

		}

		# Dictionary for OBJECTS to be recognised at a location. The x,y,theta of these locations
		# must be recorded in the locations.json file.
		self.object_dict = {
		"cardboard box"   : ["kitchen counter",
								"kitchen table",
								"coffee table",
								"bedside table"
							],	
		"coca-cola can"   : ["kitchen table"],
		"mug"             : ["kitchen counter"],
		"candle"          : ["coffee table"],
		"cup"             : ["kitchen table"],
		"reading glasses" : ["bedside table"],
		"candle"          : ["coffee table"]
		}

	def listen4cmd(self,status):
		if status == 'on' :	
			self.sub_cmd=rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)  
			print('***** Listening for a COMMAND')  
		else:
			self.sub_cmd.unregister()
			print('***** NOT! Listening for a COMMAND')  

		return

	def listen4ans(self,status):
		if status == 'on' :	
			self.sub_ans=rospy.Subscriber("/hearts/stt", String, self.hearAnswer_callback)  
			print('***** Listening for an ANSWER')
		else:
			self.sub_ans.unregister()
			print('***** NOT! Listening for an ANSWER')

		return		

	def hearAnswer_callback(self,data):	  
		speech = str(data)
		speech = speech.lower()
		rospy.loginfo('*** Heard an answer : '+speech+'\n')
		words  = speech.split(' ')

		if 'yes' in words:
			self.say("OK then I will do that")
			exec(self.code2exec)
			self.say("Task is now complete. Please give me my next instruction")
			
			# re-establish subscribers	
			self.listen4ans('off')
			self.listen4cmd('on')

		elif 'no' in words:
			self.say("OK I will forget your last instruction. Please give me my next instruction")

			# re-establish subscribers	
			self.listen4ans('off')
			self.listen4cmd('on')	

		else:
			self.say("Please answer with a yes or no")

		return

	def hearCommand_callback(self,data):

		speech = str(data)
		speech = speech.lower()
		rospy.loginfo('*** Heard a command\n'+speech+'\n')

		# remove the ROS msg "Data:" value from speech	
		item = 'data:'
		if item in speech:
			speech = speech.replace(item,'')

        # check that text has been returned
		if "bad_recognition" in speech:
			self.say("Sorry but no words were recognised please repeat your instruction")
			return 

		lookupkey = self.bld_lookupkey(speech)
		rospy.loginfo("*** lookup key: "+lookupkey)

		# obtain  "Directive to Robot" from dictionary
		self.code2exec = self.actions_dict.get(lookupkey)

		# check that lookup key was found
		if self.code2exec != None:
			#listen for "answer"
			self.say("You requested that I "+speech+' .')
			self.say("Shall I do this now")
			# get confirmation of instruction
			self.listen4cmd('off')
			self.listen4ans('on')
		else:
			self.say("Your request was not understood       please repeat")

		return
	
	def bld_lookupkey(self,speech):

		# Change words for fetching to "get"
		rspeech = speech # Reduced words in captured speech
		for sub_item in self.get_words:
			rspeech = rspeech.replace(sub_item,'get')

		# build list of words	
		words = rspeech.split(' ')	

		# change 2nd and subsequent values: 'off' to ''
		firstfound = True
		for idx, sub_item in enumerate(words):
			if sub_item == 'off' and firstfound:
				firstfound   = False
			elif sub_item == 'off' and not firstfound:
				words[idx] = ''

		#remove unwanted words
		for rm_item in self.rm_words:	
			for idx,elem in enumerate(words):
				if elem == rm_item:
					words[idx] = ''
	
		# rebuild lookup key as a string with single spaces
		lookupkey = ''
		for ii in range(1,len(words)):
			if words[ii] != '':
				lookupkey = lookupkey + words[ii]+' '
		lookupkey = lookupkey.strip()

		return lookupkey

	# exec def's for DEVICE actions
	def on_LLB(self):
		# ON  Left Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_2/on', std_srvs.srv.Empty)
		run_service()
		return

	def off_LLB(self):
		# OFF Left Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_2/off', std_srvs.srv.Empty)
		run_service()
		return	

	def on_RLB(self):
		# ON   Right Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_1/on', std_srvs.srv.Empty)
		run_service()
		return

	def off_RLB(self):
		# OFF Right Light Bedroom
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/switch_1/off', std_srvs.srv.Empty)
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

        # set light dimmer to half
	def half_L(self):
		print("\n***** in dimmer call\n")
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/dimmer/set',Percentage)
		percent = 50
 		run_service(percent) 

	
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
		run_service = rospy.ServiceProxy('/roah_rsbb/devices/blinds/set',Percentage)
		percent = 50
		run_service(percent) 
		return

	def go_home(self):
		print("\n************ write code to send me home!!\n")
		print("***** Pretend I have gone HOME (Idiling Position)!\n")
		self.move_to_location("home")
		self.say("I am going home now")
		return

	def get(self,object):
		# use object to look up location
		location  = self.object_dict.get(object)
		nlocations = len(location)
		print('n locations = '+str(nlocations))
		print('Location is = ')
		print(location)

		for LOC in location:
			print("\n***** For object : "+object+" - Location is : "+LOC+"\n")
			print("***** Go there")
			## self.move_to_location(LOC) #robot moves to corresponding position according to locations.json file in hearts_navigation
			print("***** Recognise object")
			print("***** return to GA \n")
			## Fsert lightelf.move_to_pose2D(self.user_location)
		
		
		return	

	### When receiving a message from the "roah_rsbb/benchmark/state" topic, will then publish the corresponding state to "roah_rsbb/messages_save"
	def benchmark_state_callback(self, data):
		if data.benchmark_state == BenchmarkState.STOP:
			rospy.loginfo("STOP")
		elif data.benchmark_state == BenchmarkState.PREPARE:
			rospy.loginfo("PREPARE")
			try:
				time.sleep(5)
				self.prepare() # END of PREPARE msg to service
			except:
				rospy.loginfo("Failed to reply PREPARE")
		elif data.benchmark_state == BenchmarkState.EXECUTE:
			rospy.loginfo("EXECUTE")
			self.main()
			
	def wait_for_call(self):
		rospy.loginfo("***** Waiting for call from GA")
		self.wait = False
		sub = rospy.Subscriber("/roah_rsbb/tablet/call", Empty, self.tablet_callback)
		while self.wait == False:
			rospy.sleep(5)
		print("***** Call recd from GA")	
		rospy.sleep(15)
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
		rospy.sleep(1)
		self.tts_pub.publish(text)
		rospy.sleep(5)

	def device_operationsself(self):
		pass

	def main(self):
		print ("\n***** MAIN Executing *****\n")

		#wait for call 
		self.wait_for_call()
		#request location
		self.wait_for_user_location()

		#navigate to the user's location
		self.move_to_pose2D(self.user_location)
		
		self.listen4cmd('on')

		rospy.loginfo("End of MAIN programme")



if __name__ == '__main__':
	import sys
	print("\n***")
	for p in sys.path:
		print(p)
	rospy.init_node('annies_comfort', anonymous=True)
	rospy.loginfo("annies comfort controller has started")

	controller = Controller()
	#controller.main()
	rospy.spin()