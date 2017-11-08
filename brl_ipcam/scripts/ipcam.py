#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVE R
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import base64
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

    image_pub = ''  # blank var for publisher
    bridge = ''  # blank var for CvBridge

    usr = ''
    pwd = ''
    cam_name = ''
    cam_ip = ''
    cam_type = ''

    def __init__(self, camNum):

        self.loadCameraDetails(camNum)
        # set up ROS node and publisher:
        rospy.init_node(
            'ipcam_' + self.cam_name + "_snapshot",
            anonymous=True)
        self.image_pub = rospy.Publisher(
            'ipcam/' + self.cam_name + "/snapshot",
            Image,
            queue_size=10)
        self.bridge = CvBridge()

        self.setup(self.generateCameraURL())
        self.publisher()

    def loadCameraDetails(self, camNum):
        # get basepath of this package
        rospack = rospkg.RosPack()
        basepath = rospack.get_path('brl_ipcam')

        # load password and username from file:
        with open(basepath + '/conf/passwords.json') as json_file:
            data = json.load(json_file)
        self.usr = data["user"]
        self.pwd = data["pass"]
        
        # load camera details:
        with open(basepath + '/conf/cameras.json') as json_file:
            data = json.load(json_file)
        self.cam_name = data[camNum]["name"]
        self.cam_ip = data[camNum]["ip"]
        self.cam_type = data[camNum]["type"]
                
    def generateCameraURL(self):
        camURL = self.cam_ip + self.usr + "&pwd=" + self.pwd
        if(self.cam_type == "CGIProxy"):
            camURL = camURL + "&.mjpg"
        return camURL

    def setup(self, url, user=None, password=None):
        self.url = url
        auth_encoded = base64.encodestring('%s:%s' % (user, password))[:-1]

        self.req = urllib2.Request(self.url)
        self.req.add_header(
            'Authorization',
            'Basic %s' % auth_encoded)

    def get_frame(self):
        response = urllib2.urlopen(self.req)
        img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
        frame = cv2.imdecode(img_array, 1)
        return frame

    def publisher(self):
        while not rospy.is_shutdown():
            # print time.strftime("[%H:%M:%S] New Frame", time.gmtime())
            img = self.get_frame()
            # cv2.imshow("hello", img)
            # cv2.waitKey(1)
            try:
                if img.any():
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(img, "bgr8"))
            except CvBridgeError as e:
                error = e
                # print e
