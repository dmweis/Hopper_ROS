#!/usr/bin/env python

from __future__ import division

import cv2 as cv
import rospy

from vision import vision_utils as vision

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String

tracking_enabled = False
global_prev_keypoints = None
prev_gray = None
faces_image = None
features_image = None

rospy.init_node("face_detection")
result_image_publisher = rospy.Publisher("camera/detected_faces", Image, queue_size=1)
result_compressed_image_publisher = rospy.Publisher("camera/detected_faces/compressed", CompressedImage, queue_size=1)
face_position_publisher = rospy.Publisher("camera/detected_face_position", Pose2D, queue_size=4)
face_color_publisher = rospy.Publisher("hopper/face/mode", String, queue_size=3)

last_color_selected = ""

def send_color(text):
    global last_color_selected
    if last_color_selected != text:
        face_color_publisher.publish(String(text))
        last_color_selected = text

def on_image(msg):
    global tracking_enabled
    if tracking_enabled:
        global global_prev_keypoints
        global prev_gray
        global faces_image
        global features_image

        publish_images = result_image_publisher.get_num_connections() + result_compressed_image_publisher.get_num_connections() > 0

        img = vision.ros_msg_to_image(msg)
        gray = vision.process_image(img)
        height, width = img.shape[:2]

        tracked_features_image = None
        if publish_images:
            tracked_features_image = img.copy()

        if global_prev_keypoints is None or len(global_prev_keypoints) < 1:
            faces_image = img.copy()
            features_image = img.copy()
            faces = vision.detect_faces(gray, faces_image)
            if len(faces) > 0:
                send_color("breathing:purple")
                keypoints = vision.find_features(gray, faces, features_image)
                position = vision.find_bounding_rect(keypoints, tracked_features_image)
                mapped_x = vision.map_linear(position[0], 0, width, -1.0, 1.0)
                mapped_y = vision.map_linear(position[1], 0, height, -1.0, 1.0)
                face_position_publisher.publish(Pose2D(mapped_x, mapped_y, 0.0))
            else:
                send_color("breathing:red")
                keypoints = []
        else:
            keypoints = vision.track_points_move(
                gray, prev_gray, global_prev_keypoints, tracked_features_image)

            position = vision.find_bounding_rect(keypoints, tracked_features_image)
            mapped_x = vision.map_linear(position[0], 0, width, -1.0, 1.0)
            mapped_y = vision.map_linear(position[1], 0, height, -1.0, 1.0)
            if abs(mapped_x) + abs(mapped_y) < 0.2:
                send_color("breathing:blue")
            else:
                send_color("breathing:purple")
            face_position_publisher.publish(Pose2D(mapped_x, mapped_y, 0.0))

        if publish_images:
            final_image = vision.unite_images((cv.cvtColor(gray, cv.COLOR_GRAY2BGR),
                        faces_image, features_image, tracked_features_image))
            result_image_publisher.publish(vision.image_to_ros_msg(vision.resize_image(final_image)))
            result_compressed_image_publisher.publish(vision.image_to_ros_compressed_image(vision.resize_image(final_image)))
        prev_gray = gray
        global_prev_keypoints = keypoints

def on_tracking_enabled_msg(msg):
    global tracking_enabled
    tracking_enabled = msg.data
    if not tracking_enabled:
        global global_prev_keypoints
        global prev_gray
        global faces_image
        global features_image
        global_prev_keypoints = None
        prev_gray = None
        faces_image = None
        features_image = None
        send_color("random")


rospy.Subscriber("/camera/rgb/image_color", Image, on_image, queue_size=1)
rospy.Subscriber("/hopper/face_tracking_enabled", Bool, on_tracking_enabled_msg, queue_size=10)
rospy.spin()
