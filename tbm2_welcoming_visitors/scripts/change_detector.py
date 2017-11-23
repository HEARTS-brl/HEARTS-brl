#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan


rospy.init_node("change_detector", anonymous=True)

##Publishers
p = rospy.Publisher('/scan_change', String, queue_size = 10)

#Subscribers
rospy.Subscriber('/scan', LaserScan, call_back )
previous_scan = []
threshold = 100
#store last laser scan - ranges only?
def call_back(data):
	current_scan = data.ranges
	total = 0
	if previous_scan == []:
		continue
	else:
		for i in range(len(current_scan)):
			d = abs(current_scan[i]-previous_scan[i])
			total = total+d
	rospy.loginfo(total)
	if total>threshold:
		p.publish('yes')
	else:
		p.publish('no')



rospy.spin()




#take next laser scan 

#measures difference between s1 and s2

#if difference is above threshold value

#change detected


#publish to topics /detect change

