#!/usr/bin/env python
'''
Rooms:
    kitchen, bedroom, living room, dining room, hallway
Furniture:
    bed, table lamps, couch, armchairs, chairs, tables, shelves,
Objects:
    mirror, pillows, books, cups, bowls, plates

commands:
    MOVE [forward, backward]
    TURN [left, right]
    GO to the [kitchen, bedroom, living room, dining room, hallway (room)]
    ##HERE is the [room]##
    LOOK [Up, Down, Left, Right]
    SEE the [open, closed] door BETWEEN the [room] and the [room]
    SEE the [couch, bed, chair, lamp (furniture)] in the [room]
    SEE the [color] [coke, biscuits (object)] in the [room] on the [furniture]
'''
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Pose, Twist, PoseStamped
from turtlesim.msg import Pose
from math import cos, sin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospy.rostime import Duration
from move_base_msgs.msg import MoveBaseActionFeedback

class Controller():
    def __init__(self):
        
        ### Publishers - sends out goal locations, movement/turns, and speech
        #self.pub_move = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        #self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        #self.pub_pic = rospy.Publisher('/hearts/picture', String, queue_size = 10)
        
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        self.pub_head = rospy.Publisher('/head_controller/command',JointTrajectory, queue_size = 10)
        self.pub_move = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 10)  
              
        ### Subscribers - must listen for speech commands, location
        #rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        #rospy.Subscriber("/turtle1/pose", Pose, self.currentPose_callback)
        #rospy.Subscriber("hearts/navigation/pose/location", String, self.current_pose)
        
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.current_pose_callback)
    
        self.outfolder = rospy.get_param('output_path')
        
        self.objects = ['can','bottle','cup','pillow','kettle']
        self.furniture = ['couch','bed','chair','table']
        self.rooms = ['kitchen', 'bedroom', 'living_room', 'dining_room', 'hallway']
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": self.rooms,
            "look": ["up", "down", "right", "left"],
            "here": [],
            "see": self.objects,}
            
        # Movement Parameters
        self.head_lr = 0.0
        self.head_ud = 0.0
        self.head_move_step_size = .15
        self.turn_step_size = 2
        self.move_step_size = 1
        self.handle_camera_direction(['center'])

    def hearCommand_callback(self,data):
        rospy.loginfo('Heard a command')
        raw = str(data)
        text = raw.split('~')[0]
        speech = text.split(':')[1]
        words = speech.split(' ')[1:]
        rospy.loginfo(speech)
        words = [x for x in words if x!='the']
        for w in range(len(words)):
            if words[w] in ["room","chair","cabinet"] :
                words[w] = words[w-1]+'_'+words[w]
                words.pop(w-1)
        possible_verbs = self.tbm1_commands_dict.keys()
        if words[0] in possible_verbs:
            valid_command = True
            verb = words[0]
            subject = words[1:]
            #TODO parse rooms into single words (e.g. dining room into dining_room)
        else:        #self.pub_move = rospy.Publisher('/hearts/controller/move', Twist, queue_size = 10)        

            valid_command=False
            self.pub_talk.publish("Invalid command please try again")
            return
        if verb == "move" or verb =="turn":
            self.handle_moves(verb,subject)
        elif verb == "go":
            self.handle_destinations(subject)
        elif verb == "look":
            self.handle_camera_direction(subject)
        elif verb == "see":
            self.handle_picture_taking(subject) 
        elif verb == "look":
            self.handle_camera_direction(subject)      
        return 

    def handle_picture_taking(self,subject):
        rospy.loginfo('Seen something')
        if 'between' in subject: #Door option: [open, closed] door BETWEEN [room] and [room]
            thing_id = 'door_1' #TODO add matching to IDs
            status = subject[0]
            room1 = subject[-3]
            room2 = subject[-1]
            line1 = self.linewriter('type',[thing_id,'door'])
            line2 = self.linewriter('connects',[thing_id,room1,room2])
            line3 = self.linewriter('isOpen',[thing_id,status])
            to_write = [line1,line2,line3]

        elif subject[1] in self.objects: #[color] [object] in [room] on [furniture]
            if len(subject)<5:
                self.pub_talk.publish("Invalid command please try again")
            rospy.loginfo(subject)
            thing_id = 'object_1' #TODO add matching to IDs
            position_string = self.object_position() #TODO get actual position
            self.pub_pic.publish(subject[1]+'.jpg')
            color = subject[0]
            item = subject[1]
            room = subject[3]
            furniture = subject[5]
            line1 = self.linewriter('type',[thing_id, item])
            line2 = self.linewriter('in',[thing_id, room])
            line3 = self.linewriter('on',[thing_id, furniture])
            line4 = self.linewriter('position ',[thing_id, position_string])
            line5 = self.linewriter('color',[thing_id, color])
            line6 = self.linewriter('picture',[thing_id,subject[1]+'.jpg'])
            to_write = [line1,line2,line3,line4,line5,line6]

        elif subject[0] in self.furniture: # [couch, bed, chair, lamp (furniture)] in [room]
            thing_id = 'furniture_1' # TODO matching IDs
            furniture = subject[0]
            room = subject[-1]
            line1 = self.linewriter('type',[thing_id,furniture]) 
            line2 = self.linewriter('in',[thing_id,room])
            to_write = [line1,line2]
        else:
            self.pub_talk.publish("Invalid noun please try again")
            return
        f = open(self.outfolder+'sementic_map.txt','a+')
        for line in to_write:
            f.write(line+'\n')
        f.close()


    def linewriter(self, descriptor, options_list):
        text = descriptor + '('
        for option in options_list:
            text = text+option+', '
        text = text[:-1]+').'
        return(text)




    def handle_camera_direction(self,subject):
        command = JointTrajectory()
        command.joint_names = ["head_2_joint","head_1_joint"]
        point1 = JointTrajectoryPoint()
        #point2 = JointTrajectoryPoint()
        
        point1.velocities = [0.0,0.0]
        #point2.velocities = [0]
        
        point1.time_from_start = Duration(2.0,0.0)
        rospy.loginfo(subject)
        if subject[0] == 'up':
            self.head_ud = self.head_ud + self.head_move_step_size
        if subject[0] == 'down':
            self.head_ud = self.head_ud - self.head_move_step_size
        if subject[0] == 'left':
            self.head_lr = self.head_lr + self.head_move_step_size
        if subject[0] == 'right':
            self.head_lr = self.head_lr - self.head_move_step_size
                    
        rospy.loginfo(self.head_ud)
        point1.positions = [self.head_ud, self.head_lr]
        command.points = [point1]
        self.pub_head.publish(command)
        return

    def handle_destinations(self,subject):
        rospy.loginfo('Heard a destination')
        destination_list = [x for x in subject if x!="to"]
        destination = ' '.join(destination_list)
        self.pubGoal.publish(destination)
        return

    def handle_moves(self,verb,subject):
        rospy.loginfo('Moving')
        rospy.loginfo(verb)
        rospy.loginfo(subject[0])
        rospy.loginfo(subject[1])
        if len(subject)>1:
            if subject[1] == "more":
                multiplier = 3
            if subject[1] == "less":
                multiplier = .5
        else:
            multiplier = 1
        if(verb == 'move' and subject[0] == 'forward'):
            self.send_directions(self.move_step_size*multiplier,0)
        elif(verb=='move' and subject[0] == 'backward'):
            self.send_directions(-1*self.move_step_size*multiplier,0)
        elif(verb == 'turn' and subject[0] == 'left'):
            self.send_directions(0,self.turn_step_size*multiplier)
        elif(verb == 'turn' and subject[0] == 'right'):
            self.send_directions(0,-1*self.turn_step_size*multiplier)
          
    def send_directions(self,straight,turn):
        rospy.loginfo('Sending out a direction')
        t = Twist()
        ### insert intended values for movements here to fed to navigation here ###
        t.linear.x = straight
        t.linear.y = 0
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = turn

        self.pub_move.publish(t)
        rospy.sleep(1)
    
    def object_position(self):
        ####Read in base position
        d = 1 #distance from object
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        t = self.current_pose.orientation.w
        x2 = round(x + d*cos(t),2)
        y2 = round(y + d*sin(t),2)
        z2 = .5
        l = '['+str(x2)+','+str(y2)+','+str(z2)+']'
        ####Calculate approximate object position
        #object_position = currentpose + position
        return(l)


    def current_pose_callback(self, data):
        self.current_pose = data.feedback.base_position.pose

        return




if __name__ == '__main__':
    rospy.init_node('know_my_home', anonymous=True)
    rospy.loginfo("know my home controller has started")
    controller = Controller()
    rospy.spin()
