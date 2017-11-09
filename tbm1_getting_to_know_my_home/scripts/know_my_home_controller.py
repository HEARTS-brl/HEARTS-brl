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
    HERE is the [room]
    SEE the [open, closed] door BETWEEN the [room] and the [room]
    SEE the [couch, bed, chair, lamp (furniture)] in the [room]
    SEE the [color] [coke, biscuits (object)] in the [room] on the [furniture]
'''
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, Twist, PoseStamped

class Controller():
    def __init__(self):
        
        # Publishers - sends out goal locations, movement/turns, and speech
        self.pub_task = rospy.Publisher('hearts/controller/task', String, queue_size=10)
        self.pubGoal = rospy.Publisher('hearts/navigation/goal/location', String, queue_size=10)
        self.pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)
        self.pub_pic = rospy.Publisher('/hearts/camera/snapshot', String, queue_size = 10)
        
        #self.pub_move = rospy.Publisher('/hearts/controller/move', Twist, queue_size = 10)        
        self.pub_move = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

        # Subscribers - must listen for speech commands, location
        #rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        rospy.Subscriber("/hearts/stt", String, self.hearCommand_callback)

        self.outfolder = rospy.get_param('output_folder')
        
        self.objects = ['can','bottle','cup']
        self.furniture = ['couch','bed','chair']
        self.rooms = ['kitchen', 'bedroom', 'living room', 'dining room', 'hallway']
        self.tbm1_commands_dict = {
            "move": ["forward", "backward"],
            "turn": ["right", "left"],
            "go": self.rooms,
            "look": ["up", "down", "right", "left"],
            "here": [],
            "see": self.objects,}

    def hearCommand_callback(self,data):
        w = data.split(' ')
        words = [x for x in w if x!='the']
        possible_verbs = self.tbm1_commands_dict.keys()
        if words[0] in possible_verbs:
            valid_command = True
            verb = words[0]
            subject = words[1:]
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

        print('original',s)
        print('verb',verb)
        print('subject',subject)        
        return 

    def handle_picture_taking(self,subject):
        if 'between' in subject: #Door option: [open, closed] door BETWEEN [room] and [room]
            thing_id = 'door_1' #TODO add matching to IDs
            status = subject[0]
            room1 = subject[-3]
            room2 = subject[-1]
            line1 = self.linewriter('type',[thing_id,'door'])
            line2 = self.linewriter('connects',[thing_id,room1,room2])
            line3 = self.linewriter('isOpen',[thing_id,status])
            to_write = [line1,line2,line3]

        if subject[1] in self.objects: #[color] [object] in [room] on [furniture]
            thing_id = 'object_1' #TODO add matching to IDs
            position_string = '[1, 2, 0]' #TODO get actual position
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
            line6 = self.linewriter('picture',[thing_id,thing_id+'.jpg'])
            to_write = [line1,line2,line3,line4,line5,line6]

        if subject[0] in self.furniture: # [couch, bed, chair, lamp (furniture)] in [room]
            thing_id = 'furniture_1' # TODO matching IDs
            furniture = subject[0]
            room = subject[-1]
            line1 = self.linewriter('type',[thing_id,furniture]) 
            line2 = self.linewriter('in',[thing_id,room])
            to_write = [line1,line2]

        #TODO write lines to text file
        f = open(self.outfolder+'sementic_map.txt','w+')
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
        print('this feature is not supported yet')
        return

    def handle_destinations(self,subject):
        destination_list = [x for x in subject if x!="to"]
        destination = ' '.join(destination_list)
        self.pubGoal.publish(destination)
        return

    def handle_moves(self,verb,subject):
        if(verb == 'move' and subject == 'forward'):
            self.send_directions(2,0)
        elif(verb=='move' and subject == 'backward'):
            self.send_directions(-2,0)
        elif(verb == 'turn' and subject == 'left'):
            self.send_directions(0,2)
        elif(verb == 'turn' and subject == 'right'):
            self.send_directions(0,-2)
          
    def send_directions(self,straight,turn):
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
        



if __name__ == '__main__':
    rospy.init_node('know_my_home', anonymous=True)
    rospy.loginfo("know my home controller has started")
    controller = Controller()
    rospy.spin()
