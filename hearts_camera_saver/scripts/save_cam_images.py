#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from os import path
import cv2
import numpy as np

class CameraSaver:
        
    def __init__(self, input_topic, output_path, show_feed):
        self.output_path = output_path
        self.show_feed = show_feed
        
        rospy.Subscriber(input_topic, CompressedImage, self.image_cb)
        rospy.Subscriber("/hearts/camera/snapshot", String, self.filename_cb)
        
    def image_cb(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

            if self.show_feed:
                cv2.imshow("image", image)
            else:
                #cv2.destroyAllWindows()
                pass
        except CvBridgeError, e:
            print(e)
            
    def filename_cb(self, msg):
        filepath = path.join(self.output_path, msg.data)
        if self.image is None:
            print("no camera image to save at " + filepath)
        else:
            cv2.imwrite(filepath, self.image)
            print("saved camera image at " + filepath)
            
def main():
    rospy.init_node('camera_saver')
    
    input_topic = rospy.get_param("input_topic")
    output_path = rospy.get_param("output_path")
    show_feed = rospy.get_param("show_feed") != "false"
    camera_saver = CameraSaver(input_topic, output_path, show_feed)
    
    print("Camera saver running...")
    
    rospy.spin()
    
if __name__ == '__main__':
    main()
