#!/usr/bin/env python

# https://github.com/studioimaginaire/phue

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
from std_msgs.msg import String
import json

# pose = Pose value
# location = string value

class Location():
    def __init__(self):

        # Load the location data
        self.load_dict()
       
        # Set up publishers
        self.pubCurrent = rospy.Publisher('hearts/navigation/pose/location', String, queue_size=10)
        self.pubGoal = rospy.Publisher('/hearts/navigation/goal', Pose2D, queue_size=10)

        rospy.Subscriber("hearts/navigation/goal/location", String, self.locGoal_callback)
        rospy.Subscriber("move_base_simple/current_pose", Pose, self.currentPose_callback)

        self.loop()
        self.current_location = ''
        self.current_pose = Pose()
        self.current_goal = ''

        self.find_current_location()

    def load_dict(self):
        # todo: path as ros param
        json_name = rospy.get_param('locations_json')
        with open(json_name) as json_data:
            self.dict = json.load(json_data)

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
            goal = Pose2D()
            goal.x = self.dict[location_name]["x"]
            goal.y = self.dict[location_name]["y"]
            goal.theta = self.dict[location_name]["theta"]

            return goal
        except:
            return None

    # Get the location value from the Pose lookup
    def find_current_location(self):

        # @TODO get current pose

        currentPose = self.current_pose
        location = ''
        for i in self.dict:
            if (self.dict[i]["x"] == currentPose.position.x and self.dict[i]["y"] == currentPose.position.y):
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
