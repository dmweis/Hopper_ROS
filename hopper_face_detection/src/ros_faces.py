#!/usr/bin/env python

from __future__ import division

import cv2 as cv
import rospy

from vision import vision_utils as vision

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Pose2D


global_prev_keypoints = None
prev_gray = None
faces_image = None
features_image = None

rospy.init_node("face_detection")
result_image_publisher = rospy.Publisher("camera/detected_faces", Image, queue_size=1)
result_compressed_image_publisher = rospy.Publisher("camera/detected_faces/compressed", CompressedImage, queue_size=1)
face_position_publisher = rospy.Publisher("camera/detected_face_position", Pose2D, queue_size=4)

def on_image(msg):
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
        keypoints = vision.find_features(gray, faces, features_image)
    else:
        keypoints = vision.track_points_move(
            gray, prev_gray, global_prev_keypoints, tracked_features_image)
        position = vision.find_bounding_rect(keypoints, tracked_features_image)
        mapped_x = vision.map_linear(position[0], 0, width, -1.0, 1.0)
        mapped_y = vision.map_linear(position[1], 0, height, -1.0, 1.0)
        face_position_publisher.publish(Pose2D(mapped_x, mapped_y, 0.0))

    if publish_images:
        final_image = vision.unite_images((cv.cvtColor(gray, cv.COLOR_GRAY2BGR),
                    faces_image, features_image, tracked_features_image))
        # finished_image.publish(vision.image_to_ros_msg(vision.resize_image(final_image)))
        result_image_publisher.publish(vision.image_to_ros_msg(final_image))
        result_compressed_image_publisher.publish(vision.image_to_ros_compressed_image(final_image))
    prev_gray = gray
    global_prev_keypoints = keypoints

rospy.Subscriber("/camera/rgb/image_color", Image, on_image, queue_size=1)
rospy.spin()
