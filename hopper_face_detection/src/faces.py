#!/usr/bin/env python

from __future__ import division

import cv2 as cv
from vision import vision_utils as vision

global_prev_keypoints = None
prev_gray = None
faces_image = None
features_image = None
cam = cv.VideoCapture(0)
while True:
    ret, img = cam.read()
    height, width = img.shape[:2]
    gray = vision.process_image(img)

    tracked_features_image = img.copy()

    if global_prev_keypoints is None or len(global_prev_keypoints) < 1:
        faces_image = img.copy()
        features_image = img.copy()
        faces = vision.detect_faces(gray, faces_image)
        keypoints = vision.find_features(gray, faces, features_image)
    else:
        keypoints = vision.track_points_move(
            gray,
            prev_gray,
            global_prev_keypoints,
            tracked_features_image)
        vision.find_bounding_rect(keypoints, tracked_features_image)

    united_image = vision.unite_images(
        (cv.cvtColor(gray, cv.COLOR_GRAY2BGR),
         faces_image,
         features_image,
         tracked_features_image))
    key = vision.display_image(united_image, "Merged")
    prev_gray = gray
    global_prev_keypoints = keypoints

    if key % 256 == 27:
        # ESC was pressed
        break

cam.release()
cv.destroyAllWindows()
