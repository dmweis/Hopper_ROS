#!/usr/bin/env python

from __future__ import division

import cv2 as cv

def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_alt2.xml')
face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('data/haarcascade_eye.xml')

cam = cv.VideoCapture(0)
while True:
    ret, img = cam.read()
    height, width = img.shape[:2]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 2)
    for (x, y, w, h) in faces:
        mapped_x = map_linear(x+w/2, 0, width, 0.0, 1.0)
        mapped_y = map_linear(y+h/2, 0, height, 0.0, 1.0)
        print "Face at: ", mapped_x, mapped_y
        #cv.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        #roi_gray = gray[y:y+h, x:x+w]
        #roi_color = img[y:y+h, x:x+w]
        #eyes = eye_cascade.detectMultiScale(roi_gray)
        #for (ex, ey, ew, eh) in eyes:
        #    cv.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
    if len(faces) == 0:
         print "No faces"
    #cv.imshow('img', img)
    #key = cv.waitKey(1)
    #if key%256 == 27:
        # ESC was pressed
        #break

cam.release()
cv.destroyAllWindows()

