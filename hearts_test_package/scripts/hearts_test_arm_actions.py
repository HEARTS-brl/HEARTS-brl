#! /usr/bin/env python
import roslib
import rospy
from std_msgs.msg import String
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

#if __name__ == '__main__':


def action(name):
    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    client.wait_for_server()
    
    goal = PlayMotionGoal()
    goal.motion_name = name
    goal.skip_planning = False
    goal.priority = 0  # Optional

    rospy.loginfo("Sending goal with motion: " + name)
    client.send_goal(goal)
    action_ok = client.wait_for_result(rospy.Duration(30.0))

rospy.init_node('play_motion_node')


pub_talk = rospy.Publisher('/hearts/tts', String, queue_size = 10)

rospy.sleep(2)

pub_talk.publish("Hello there! ")
action("shake_hands")
pub_talk.publish("How are you today?")
rospy.sleep(5)
pub_talk.publish("It was a pleasure to meet you")
action("wave")
pub_talk.publish("Goodbye!")

rospy.loginfo("Waiting for result...")

       
