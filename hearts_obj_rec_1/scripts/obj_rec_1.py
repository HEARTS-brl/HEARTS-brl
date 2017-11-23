#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String, UInt8
#from roah_rsbb_comm_ros.msg import Benchmark, BenchmarkState
#from geometry_msgs.msg import Pose2D
#import std_srvs.srv
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from os import mkdir, path
import cv2
import re
import cloudsight
from math import pow, sqrt
from tf import StampedTransform, TransformListener

#import urllib3
#urllib3.disable_warnings()

#see:
#https://github.com/pal-robotics/aruco_ros/blob/indigo-devel/aruco_ros/src/simple_single.cpp
#http://wiki.ros.org/Robots/TIAGo/Tutorials/MoveIt/Pick_place

class ImageRegion():
    def __init__(self, region, start_x, start_y, end_x, end_y, name, distance_from_center):
        self.region = region
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.name = name
        self.distance_from_center = distance_from_center
           
class Comms():

    def __init__(self):
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.label_sub = rospy.Subscriber("/hearts/obj_rec/label", String, self.label_callback)
        self.camera_info_sub = rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_callback)

        self.listener = TransformListener()
                
        self.reference_frame = rospy.get_param("reference_frame", "")
        
        if not self.listemer.frameExists(self.reference_frame):
            rospy.logerr("invalid reference frame: " + self.reference_frame)
            
        self.camera_frame = rospy.get_param("camera_frame", "")

        if not self.listemer.frameExists(self.camera_frame):
            rospy.logerr("invalid camera frame: " + self.camera_frame)
            
        self.temp_file_path = "/tmp/obj_rec_1/"
        
        self.bridge = CvBridge()
        self.match = re.compile("name': u'(.*?)',")

        self.h_margin = 100;
        self.v_margin = 100;
        
        self.n_attempts = 1
        self.n_h_regions = 4
        self.n_v_regions = 3
    
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
	    
        self.auth = cloudsight.OAuth('t-xw3Mn06dkObZwsi8qpcA', 'laJ83zYRxAVpls1Aera8Mg')
        self.api = cloudsight.API(self.auth)
        
        self.image_regions = None

    def recognise(self, file_path, n_attempts):
		
        i = 0
        while i < n_attempts:
		
            with open(file_path, 'rb') as f:
                response = self.api.image_request(f, file_path, { 'image_request[locale]': 'en-US', })

            status = self.api.image_response(response['token'])
	        
            while status['status'] == cloudsight.STATUS_NOT_COMPLETED:
                time.sleep(5)
                status = self.api.image_response(response['token'])
                rospy.loginfo('waiting....')	        
	        
                try:
                    return re.search(self.match, str(status)).group(1)
                except:
                    i = i + 1

        return None

    def image_callback(self, msg):
        # split camera image into n overlapping regions
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = image.shape[:2]
            h_2 = h / 2
            w_2 = w / 2
            
            region_h = (h - (2*self.v_margin)) / self.n_v_regions
            region_w = (w - (2*self.h_margin)) / self.n_h_regions
            
            region_h_2 = region_h / 2
            region_w_2 = region_w / 2
            
            image_regions = [ ]
            
            for i in range(0, self.n_v_regions):
                for j in range(0, self.n_h_regions):
                    start_y = (region_h * i) + self.v_margin
                    start_x = (region_w * j) + self.h_margin
                    end_y = start_y + region_h
                    end_x = start_x + region_w
                    center_y = start_y + region_h_2
                    center_x = start_x + region_w_2
                    distance_from_center = sqrt(pow(h_2 - center_y, 2) + pow(w_2 - center_x, 2))
                    image_region_index = (i * self.n_v_regions) + j
                    name = "region " + str(image_region_index)
                    region = image[start_y:end_y, start_x:end_x]
                    image_region = ImageRegion(region, start_x, start_y, end_x, end_y, name, distance_from_center)
                    image_regions.append(image_region)
                    
                    cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (255,0,0), 1)
            
            n_off_v_regions = self.n_v_regions - 1
            n_off_h_regions = self.n_h_regions - 1
            
            for i in range(0, n_off_v_regions):
                for j in range(0, n_off_h_regions):
                    start_y = (region_h * i) + self.v_margin + region_h_2
                    start_x = (region_w * j) + self.h_margin + region_w_2
                    end_y = start_y + region_h
                    end_x = start_x + region_w
                    center_y = start_y + region_h_2
                    center_x = start_x + region_w_2
                    distance_from_center = sqrt(pow(h_2 - center_y, 2) + pow(w_2 - center_x, 2))
                    image_region_index = (i * n_off_v_regions) + j
                    name = "region " + str(image_region_index) + " off"
                    region = image[start_y:end_y, start_x:end_x]
                    image_region = ImageRegion(region, start_x, start_y, end_x, end_y, name, distance_from_center)
                    image_regions.append(image_region)
                    
                    cv2.rectangle(image, (start_x, start_y), (end_x, end_y), (0,0,255), 1)
            
            cv2.imshow("image", image)
            cv2.waitKey(1)
                    
            self.image_regions = sorted(image_regions, key=self.get_distance_from_center)
            
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_distance_from_center(self, image_region):
        return image_region.distance_from_center
        
    def label_callback(self, msg):
        # accept name of object to recognise
       
        label = msg.data.lower()
        rospy.loginfo("received label: " + label)
        
        image_regions = self.image_regions
        
        if not path.exists(self.temp_file_path):
            mkdir(self.temp_file_path)
            
        # save image regions
        for image_region in image_regions:
            file_name = image_region.name + ".jpg"
            file_path = path.join(self.temp_file_path, file_name)
            cv2.imwrite(file_path, image_region.region)
        
        # send regions to cloudsight, starting in the center    
        #image_regions_for_label = [ ]
        
        for image_region in image_regions:
            file_name = image_region.name + ".jpg"
            file_path = path.join(self.temp_file_path, file_name)
            result = self.recognise(file_path, self.n_attempts) 
            
            if not result is None:
                rospy.loginfo("cloudsight result: " + result)
                tokens = [ token.lower() for token in re.split(r'-+| +|', result) ]
                if label in tokens:
                    image_region.start_x, image_region.start_y, image_region.end_x, image_region.end_y
                    break
                    #image_regions_for_label.append()
        
        print("found " + str(len(image_regions_for_label)) + " regions!")
                    
        # aggregate regions with labels similar to name
        
        # get transformation between reference frame and camera frame
        
        if self.reference_frame != self.camera_frame:
            (position, orientation) = self.get_transform(self.reference_frame, self.camera_frame)
        
        # publish coordinates of aggregated image region (and original image size?)               

    def camera_info_callback(self, msg):
        # todo; what do I do with the camera info?
        self.right_to_left = StampedTransform()
        self.right_to_left.setIdentity()
        
    
    def get_transform(self, ref_frame, child_frame):
        now = rospy.Time.now()
        try:
            if self.listener.waitForTransform(refFrame, childFrame, now, rospy.Duration(5.0))
                return self.listener.lookupTransform(refFrame, childFrame, now)
        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.logerr(e)
        return None
        
    def CropImageFileCallback(self, data):
		
		#filename = "/home/kaya/hearts/src/hearts/object_perception_2/data/Vaseline_1.jpg"
		file_paths = data.data.split(',')

		for i in range(0, len(file_paths)):
			#for j in range(0, 1): # self.max_attempts
			file_path = file_paths[i]
			rospy.loginfo("Trying image " + str(i))
			rospy.loginfo("Filepath: " + file_path)
			cloud_return = self.recognise(file_path)
			if cloud_return == 1:
				rospy.loginfo('Failed to identify object (cloud error)')
				continue		
			class_name, pub_class_name, object_name, pub_object_name = cloud_return 
			if pub_class_name == 'unknown' and pub_object_name == 'unknown':
				rospy.loginfo('Failed to identify object (no match to item dictionary)')
                                #self.pubStatus.publish("bad")
				continue
			elif pub_object_name == 'unknown':
				pub_object_name = pub_class_name + '1'
			elif pub_class_name == 'unknown':
				pub_class_name = result = ''.join([i for i in pub_object_name if not i.isdigit()])
			rospy.loginfo('identified object - Class: ' + class_name + ' (' + pub_class_name + ') , Object: ' + object_name  + ' (' + pub_object_name + ')')
			self.pubGoal.publish(pub_class_name + ',' + pub_object_name)
			rospy.loginfo(':) Another Object Please')
			self.image_attempts = 0
			return 1

		if self.image_attempts < 2:
			rospy.loginfo('Can\'t identify image. Take another one please!')
			self.pubStatus.publish('try again')
			self.image_attempts = self.image_attempts + 1
		else:
			rospy.loginfo('Tried ' + str(self.max_attempts) + ' times. Can\'t identify. Try another object.')
			self.pubGoal.publish('unknown,unknown') 
			self.image_attempts = 0

if __name__ == '__main__':
    rospy.init_node('cloudsight_objectperception', anonymous=True)
    rospy.loginfo("CLOUDSIGHT: Started")
    c = Comms()
    rospy.spin()

