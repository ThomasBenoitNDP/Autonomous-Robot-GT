#!/usr/bin/python

import roslib
roslib.load_manifest('face_detect')

import sys
import os

import rospy
import sensor_msgs.msg
from visualization_msgs.msg import Marker
from sensor_msgs.msg import *
from face_detect.msg import *



pub = rospy.Publisher('visualization_marker', Marker, queue_size = 1)

rospy.init_node('Marker', anonymous=True)

marker = Marker()
   

#Rate at which the loop will repeat
rate = rospy.Rate(rospy.get_param('~hz', 10))

def callback(msg):
    if len(msg.ROIlist) == 0:
        marker.text = "F"
        marker.scale.x = 2e-15
        marker.scale.y = 2e-15
        marker.scale.z = 2e-15
    else:
        marker.text = "Face detected"
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
	
	   
sub = rospy.Subscriber('face_detect/ROI', RegionOfInterestArray, callback)


while not rospy.is_shutdown():
	
    marker.header.frame_id = "/base_link"
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
   
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
   
    marker.pose.position.x = 1.5
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.w = 1.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
      
    pub.publish(marker)
	
	#Waiting to repeat the loop
    rate.sleep()

