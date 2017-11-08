#!/usr/bin/env python

import base64
import time
import urllib2

import cv2
import numpy as np

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import json


"""
Examples of objects for image frame aquisition from both IP and
physically connected cameras

Requires:
 - opencv (cv2 bindings)
 - numpy
"""

class ipCamera(object):

    def __init__(self, url, user=None, password=None):
        self.url = url
        auth_encoded = base64.encodestring('%s:%s' % (user, password))[:-1]

        self.req = urllib2.Request(self.url)
        self.req.add_header('Authorization', 'Basic %s' % auth_encoded)

    def get_frame(self):
        response = urllib2.urlopen(self.req)
        img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
        frame = cv2.imdecode(img_array, 1)
        return frame

def main(camNum, videoMode):
    # living room - 192.168.1.72
    # bedroom - 192.168.1.74
    # kitchen - 192.168.1.75
    # hall - 192.168.1.78

    # load password and username from file:
    rospack = rospkg.RosPack()
    basepath = rospack.get_path('brl_ipcam')
    with open(basepath + '/conf/passwords.json') as json_file:
        data = json.load(json_file)
        #print data
    usr = data["user"]
    pwd = data["pass"]


    rospy.init_node('ipcam_' + videoMode + "_" + camNum, anonymous=True)
    image_pub = rospy.Publisher('ipcam/' + videoMode + "/" + camNum ,Image)
    bridge = CvBridge()

##### SNAPSHOT Mode:
    if videoMode == "SS":

        # snapshot url:
        camURL = {  "1" : "http://192.168.1.78/snapshot.cgi?user=" + usr + "&pwd=" + pwd,
                    "2" : "http://192.168.1.72:88/cgi-bin/CGIProxy.fcgi?cmd=snapPicture2&usr=" + usr + "&pwd=" + pwd +"&.mjpg",
                    "3" : "http://192.168.1.75/snapshot.cgi?user=" + usr + "&pwd=" + pwd,
                    "4" : "http://192.168.1.74:88/cgi-bin/CGIProxy.fcgi?cmd=snapPicture2&usr=" + usr + "&pwd=" + pwd +"&.mjpg"
                    }
        cam = ipCamera(camURL[camNum])

        while not rospy.is_shutdown():
            #print time.strftime("[%H:%M:%S] New Frame", time.gmtime())
            img = cam.get_frame()
            #cv2.imshow("hello", img)
            #cv2.waitKey(1)
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            except CvBridgeError as e:
                error = e
                #print e

##### VIDEO STREAM Mode
# @TODO: Make this work.
    elif videoMode == "VS":


        camURL = {  "1" : "http://192.168.1.78/videostream.cgi?user" + usr + "&pwd=" + pwd,
                    "2" : "http://192.168.1.72:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=" + usr + "&pwd=" + pwd +"&.mjpg",
                    "3" : "http://192.168.1.75/videostream.cgi?user" + usr + "&pwd=" + pwd,
                    "4" : "http://192.168.1.74:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=" + usr + "&pwd=" + pwd +"&.mjpg"
                    }

        cap = cv2.VideoCapture(camURL[camNum])
        while not rospy.is_shutdown():
            _, img = cap.read()
            if img is not None:
                #print time.strftime("[%H:%M:%S] New Frame", time.gmtime())
                cv2.imshow("hello", img)
                #cv2.waitKey(1)
                try:
                  image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
                except CvBridgeError as e:
                  error = e
                  #print(e)

    cv2.destroyAllWindows()

# run code if specifically executed as a program, not an import
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: ipcam.py camera_number[1-4] video_mode[SS or VS]")
    else:
        main(sys.argv[1], sys.argv[2])
