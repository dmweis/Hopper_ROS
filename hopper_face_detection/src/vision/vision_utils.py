from __future__ import division

import os
import cv2 as cv
import numpy as np

from cv_bridge import CvBridge


def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def process_image(img, equalize_hist=False):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    if equalize_hist:
        gray = cv.equalizeHist(gray)
    return gray

cv_bridge=None

def ros_msg_to_image(msg):
    global cv_bridge
    if cv_bridge is None:
        cv_bridge = CvBridge()
    return cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def image_to_ros_msg(image):
    global cv_bridge
    if cv_bridge is None:
        cv_bridge = CvBridge()
    return cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")

script_dir = os.path.dirname(os.path.realpath(__file__))
#face_cascade = cv.CascadeClassifier('data/haarcascade_frontalface_alt2.xml')
face_cascade = cv.CascadeClassifier(
    script_dir + '/data/haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier(script_dir + '/data/haarcascade_eye.xml')

def detect_faces(gray, image_to_draw_on=None, detect_eyes=False):
    global face_cascade
    faces = face_cascade.detectMultiScale(gray, 1.3, 3)
    if image_to_draw_on is not None:
        for (x, y, w, h) in faces:
            cv.rectangle(image_to_draw_on, (x, y), (x+w, y+h), (255, 0, 0), 2)
            if detect_eyes:
                face_roi_gray = gray[y:y+h, x:x+w]
                face_roi_color = img[y:y+h, x:x+w]
                detect_eyes(face_roi_gray, face_roi_color)
    return faces


def detect_eyes(gray, image):
    global eye_cascade
    eyes = eye_cascade.detectMultiScale(roi_gray, 1.1, 2)
    for (ex, ey, ew, eh) in eyes:
        cv.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
    return eyes


def get_biggest_face(faces):
    return sorted(faces, key=lambda (x, y, w, h): w*h, reverse=True)[0]


def find_features(gray, faces, image_to_draw_on=None):
    # create masking image that is all black
    face_mask = np.zeros_like(gray)
    keypoints = []
    if len(faces) > 0:
        x, y, w, h = get_biggest_face(faces)
        # set face ROI to white
        face_mask[y:y+h, x:x+h] = 255
        corners = cv.goodFeaturesToTrack(gray, 200, 0.02, 7, mask=face_mask)
        corners = np.int0(corners)
        for corner in corners:
            x, y = corner.ravel()
            keypoints.append((x, y))
            if image_to_draw_on is not None:
                cv.circle(image_to_draw_on, (x, y), 3, 255, -1)
    return keypoints


def track_points_move(gray, previous_gray, keypoints, image_to_draw_on=None):
    re_keypoints = np.float32([p for p in keypoints]).reshape(-1, 1, 2)
    optical_flow, st, err = cv.calcOpticalFlowPyrLK(previous_gray,
                                                    gray,
                                                    re_keypoints,
                                                    None,
                                                    winSize=(10, 10),
                                                    maxLevel=2,
                                                    criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 20, 0.01))

    optical_flow_reversed, st, err = cv.calcOpticalFlowPyrLK(gray,
                                                             previous_gray,
                                                             optical_flow,
                                                             None,
                                                             winSize=(10, 10),
                                                             maxLevel=2,
                                                             criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 20, 0.01))
    distances = abs(re_keypoints-optical_flow_reversed).reshape(-1, 2).max(-1)
    good_points = distances < 1
    new_keypoints = []
    for (x, y), good_flag in zip(optical_flow.reshape(-1, 2), good_points):
        if not good_flag:
            continue
        new_keypoints.append((x, y))
        if image_to_draw_on is not None:
            cv.circle(image_to_draw_on, (x, y), 3, 255, -1)
    return new_keypoints

def find_bounding_rect(keypoints, image_to_draw_on=None):
    bounding_rectangle = cv.boundingRect(np.float32(keypoints).reshape(-1, 1, 2))
    top_left = bounding_rectangle[:2]
    bottom_right = (top_left[0] + bounding_rectangle[2],
                    top_left[1] + bounding_rectangle[3])
    if image_to_draw_on is not None:
        cv.rectangle(image_to_draw_on, top_left, bottom_right, 255, 4)
    center_x = top_left[0] + bounding_rectangle[2] / 2.0
    center_y = top_left[1] + bounding_rectangle[3] / 2.0
    return (center_x, center_y)

def unite_images(images):
    img1 = cv.hconcat(images[:2])
    img2 = cv.hconcat(images[2:])
    img = cv.vconcat((img1, img2))
    return img

def resize_image(image, width=320, height=240):
    return cv.resize(image, (width, height))

def display_image(image, text="image"):
    cv.imshow(text, image)
    return cv.waitKey(1)
