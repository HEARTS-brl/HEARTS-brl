#!/usr/bin/env python

# https://github.com/studioimaginaire/phue

import rospy
import time
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import json

# pose = Pose value
# location = string value

class Location():
    def __init__(self):

        # Load the location data
        self.locations_file_path = '~/tb_ws/src/brl-hearts/hearts_navigation/data/locations.json'
        self.load_dict()
        self.var = rospy.Time.now()

        # Set up publishers
        self.pubCurrent = rospy.Publisher('hearts/navigation/pose/location', String, queue_size=10)
        self.pubGoal = rospy.Publisher('/hearts/navigation/goal', PoseStamped, queue_size=10)

        rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        rospy.Subscriber("move_base_simple/current_pose", Pose, self.currentPose_callback)

        self.loop()
        self.current_location = ''
        self.current_pose = Pose()
        self.current_goal = ''

        self.find_current_location()

    def load_dict(self):
        self.dict = json.load(open(self.locations_file_path))

    def clicked_callback(self, data):
        print data.data

        p = {
            "location_name":
                {
                "header": {
                    "seq": 0,
                    "stamp": {
                        "secs": 0,
                        "nsecs": 0
                    },
                    "frame_id": "/map"
                },
                "pose": {
                    "position": {
                        "x": 0,
                        "y": 0,
                        "z": 0
                    },
                    "orientation": {
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "w": 0
                    }
                }
            }
        }

    # When a location is published, turn it in to a pose goal
    def locGoal_callback(self, data):
        print data.data
        self.current_goal = data.data
        pose = self.find_pose(data.data)
        if not pose is None:
            self.pubGoal.publish(pose)

    # When a pose is published, convert it to a location value
    def currentPose_callback(self, data):
        print data.data
        self.current_pose = data.data
        self.pubCurrent.publish(self.find_current_location())

    # Get the pose values from the location dict
    def find_pose(self, location_name):

        try:
            print self.dict[location_name]
            goal = PoseStamped()

            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "/map"

            goal.pose.position.x = self.dict[location_name]["pose"]["position"]["x"]
            goal.pose.position.y = self.dict[location_name]["pose"]["position"]["y"]
            goal.pose.position.z = self.dict[location_name]["pose"]["position"]["z"]

            goal.pose.orientation.x = self.dict[location_name]["pose"]["orientation"]["x"]
            goal.pose.orientation.y = self.dict[location_name]["pose"]["orientation"]["y"]
            goal.pose.orientation.z = self.dict[location_name]["pose"]["orientation"]["z"]
            goal.pose.orientation.w = self.dict[location_name]["pose"]["orientation"]["w"]

            return goal
        except:
            return None

    # Get the location value from the Pose lookup
    def find_current_location(self):

        # @TODO get current pose

        currentPose = self.current_pose
        location = ''
        for i in self.dict:
            if(self.dict[i]["pose"]["position"] == currentPose):
                location = i

        return location

    # loop
    def loop(self):

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("task_controller", anonymous=True)
    loc = Location()
    rospy.spin()
