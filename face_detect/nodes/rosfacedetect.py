#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib
roslib.load_manifest('face_detect')

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import *
from face_detect.msg import *

min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True



if __name__ == '__main__':
    opencv_dir = '/usr/share/opencv/haarcascades/';

    face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print "Could not find face cascade"
        sys.exit(-1)
    eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
    if eye_cascade.empty():
        print "Could not find eye cascade"
        sys.exit(-1)
    br = CvBridge()
    pub = rospy.Publisher('face_detect/ROI', RegionOfInterestArray, queue_size=1)
    rospy.init_node('face_detect')
        
    display = rospy.get_param("~display",True)

    def detect_and_draw(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        #We store the number of detected faces
        nb_faces = len(faces)
        #We create a list that stores all the msgs for the detected faces
        ROI = [RegionOfInterest()] * nb_faces
        #We create a message with a dynamic list that can be published
        #This message had to be created in order to publish many ROI at the same time
        #because the default RegionOfInterest message does not handle it.
        multiROI = RegionOfInterestArray()
        
        #We input the values of xoffset, yoffset, width and length depending
        #on the number of faces detected
        for i in range(nb_faces):
            ROI[i].x_offset = faces[i][0]
            ROI[i].y_offset = faces[i][1]            
            ROI[i].width = faces[i][2]
            ROI[i].height = faces[i][3]
        
        #We push all the msgs created into our own multiROI msg  
        multiROI.ROIlist = ROI    
        
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
                
        cv2.imshow('img',img)
        cv2.waitKey(10)
        #We publish our multiROI msg
        pub.publish(multiROI)

    
    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_draw)
    rospy.spin()
