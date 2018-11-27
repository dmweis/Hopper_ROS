#!/usr/bin/env python

from __future__ import division

import rospy
import os
import cv2 as cv

from geometry_msgs.msg import Vector3

def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

rospy.init_node("face_detector")
pub = rospy.Publisher("hopper/detected_faces", Vector3, queue_size=2)
#face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_alt2.xml')
face_cascade = cv.CascadeClassifier(os.path.dirname(os.path.realpath(__file__)) + '/data/haarcascade_frontalface_default.xml')
#eye_cascade = cv.CascadeClassifier('data/haarcascade_eye.xml')

cam = cv.VideoCapture(0)
while not rospy.is_shutdown():
    ret, img = cam.read()
    height, width = img.shape[:2]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 2)
    for (x, y, w, h) in faces:
        mapped_x = map_linear(x+w/2, 0, width, -1.0, 1.0)
        mapped_y = map_linear(y+h/2, 0, height, -1.0, 1.0)
        pub.publish(Vector3(mapped_x, mapped_y, 0))
        #cv.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
        #for (ex, ey, ew, eh) in eyes:
        #    cv.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
    if len(faces) == 0:
         pub.publish(Vector3(0, 0, 0))
    #cv.imshow('img', img)
    #key = cv.waitKey(1)
    #if key%256 == 27:
        # ESC was pressed
        #break

cam.release()
cv.destroyAllWindows()

