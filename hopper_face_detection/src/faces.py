#!/usr/bin/env python

from __future__ import division

import cv2 as cv
import numpy as np


def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


#face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_alt2.xml')
face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('data/haarcascade_eye.xml')


def process_image(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # gray = cv.equalizeHist(gray_original)
    return gray


def detect_faces(gray, imgae_to_draw_on):
    faces = face_cascade.detectMultiScale(gray, 1.3, 3)
    for (x, y, w, h) in faces:
        # mapped_x = map_linear(x+w/2, 0, width, 0.0, 1.0)
        # mapped_y = map_linear(y+h/2, 0, height, 0.0, 1.0)
        # print "Face at: ", mapped_x, mapped_y
        cv.rectangle(imgae_to_draw_on, (x, y), (x+w, y+h), (255, 0, 0), 2)
        # roi_gray = gray[y:y+h, x:x+w]
        # roi_color = img[y:y+h, x:x+w]
        # eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 2)
        # for (ex, ey, ew, eh) in eyes:
        #    cv.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
    return faces


def find_features_in_faces(gray, faces, image_to_draw_on):
    face_mask = np.zeros_like(gray)
    keypoints = []
    if len(faces) > 0:
        x, y, w, h = faces[0]
        face_mask[y:y+h, x:x+h] = 255
        corners = cv.goodFeaturesToTrack(gray, 200, 0.02, 7, mask=face_mask)
        corners = np.int0(corners)
        for i in corners:
            x, y = i.ravel()
            keypoints.append((x, y))
            cv.circle(image_to_draw_on, (x, y), 3, 255, -1)
    return keypoints


def track_points_move(gray, prev_gray, keypoints, img_to_draw):
    # reshape keypoints
    re_keypoints = np.float32([p for p in keypoints]).reshape(-1, 1, 2)
    optical_flow, st, err = cv.calcOpticalFlowPyrLK(prev_gray, gray,
                                                    re_keypoints,
                                                    None,
                                                    winSize=(10, 10),
                                                    maxLevel=2,
                                                    criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 20, 0.01))

    optical_flow_reversed, st, err = cv.calcOpticalFlowPyrLK(gray, prev_gray,
                                                             optical_flow,
                                                             None,
                                                             winSize=(10, 10),
                                                             maxLevel=2,
                                                             criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 20, 0.01))
    distances = abs(re_keypoints-optical_flow_reversed).reshape(-1, 2).max(-1)

    good = distances < 1
    new_keypoints = []

    for (x, y), good_flag in zip(optical_flow.reshape(-1, 2), good):
        if not good_flag:
            continue
        new_keypoints.append((x, y))
        cv.circle(img_to_draw, (x, y), 3, 255, -1)
    op_flow_array = np.float32([p for p in optical_flow]).reshape(-1, 1, 2)
    if len(optical_flow) > 6 and False:
        ellipse = cv.fitEllipse(op_flow_array)
        cv.ellipse(img_to_draw, ellipse, 255, 4)
    else:
        rect = cv.boundingRect(op_flow_array)
        top_left = rect[:2]
        bottom_right = (top_left[0] + rect[2], top_left[1] + rect[3])
        cv.rectangle(img_to_draw, top_left, bottom_right, 255, 4)
    return new_keypoints


def display_images(images):
    img1 = cv.hconcat(images[:2])
    img2 = cv.hconcat(images[2:])
    img = cv.vconcat((img1, img2))
    cv.imshow("images", img)


global_prev_keypoints = None
prev_gray = None
faces_image = None
features_image = None
cam = cv.VideoCapture(0)
while True:
    ret, img = cam.read()
    height, width = img.shape[:2]
    gray = process_image(img)

    tracked_features_image = img.copy()

    if global_prev_keypoints is None or len(global_prev_keypoints) < 1:
        faces_image = img.copy()
        features_image = img.copy()
        faces = detect_faces(gray, faces_image)
        keypoints = find_features_in_faces(gray, faces, features_image)
    else:
        keypoints = track_points_move(
            gray, prev_gray, global_prev_keypoints, tracked_features_image)

    display_images((cv.cvtColor(gray, cv.COLOR_GRAY2BGR),
                    faces_image, features_image, tracked_features_image))
    prev_gray = gray
    global_prev_keypoints = keypoints

    key = cv.waitKey(1)
    if key % 256 == 27:
        # ESC was pressed
        break

cam.release()
cv.destroyAllWindows()
