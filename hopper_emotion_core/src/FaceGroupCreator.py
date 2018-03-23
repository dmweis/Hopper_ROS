#!/usr/bin/env python

from __future__ import print_function
from io import BytesIO
from time import time
import rospy
import cognitive_face
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from hopper_emotion_core.msg import DetectedFace, FaceRectangle, FaceAttributes, Emotions, FaceDetectionImage

# disable warnings on unsecure requests done by the MS face api library
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)


class FaceGroupCreator(object):
    def __init__(self):
        print("Starting node")
        rospy.init_node("Face_group_creator", anonymous=True)
        api_key = rospy.get_param("face_api_key")
        api_url = rospy.get_param("face_api_base_url")
        cognitive_face.Key.set(api_key)
        cognitive_face.BaseUrl.set(api_url)
        self.face_counter = 0
        self.face_counter_max = 1
        # init person group
        self.person_group_id = "primary_user_group"
        # cognitive_face.person_group.delete(self.person_group_id)
        # cognitive_face.person_group.create(self.person_group_id)
        # print("Created person group")
        # init first user
        person_name = "Dominika"
        self.person_id = cognitive_face.person.create(self.person_group_id, person_name)
        print("Perissted person id for " + str(person_name) + " is " + str(self.person_id))
        self.persisted_face_ids = []
        # init subscribe
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.new_image_callback, queue_size=4)
        rospy.spin()

    def new_image_callback(self, compressed_image):
        print("Received image")
        if self.face_counter < self.face_counter_max:
            self.face_counter += 1
            with BytesIO(compressed_image.data) as image_stream:
                new_persistent_face_id = cognitive_face.person.add_face(image_stream, self.person_group_id, self.person_id['personId'])
                print("Added new face under: " + str(new_persistent_face_id))
                self.persisted_face_ids.append(new_persistent_face_id)
        else:
            print(self.persisted_face_ids)
            print("Starting training")
            cognitive_face.person_group.train(self.person_group_id)
            cognitive_face.util.wait_for_person_group_training(self.person_group_id)
            print("Training finished")
            rospy.signal_shutdown("Program finished")


if __name__ == "__main__":
    FaceGroupCreator()
