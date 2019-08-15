#!/usr/bin/env python

import rospy

from hopper_emotion.msg import FaceDetectionImage
from std_msgs.msg import String
from random import choice


def get_strongest_emotion(detected_face):
    emotions = detected_face.face_attributes.emotions
    emotions_dict = {}
    emotions_dict[emotions.sadness] = "sad"
    emotions_dict[emotions.neutral] = "neutral"
    emotions_dict[emotions.contempt] = "contempt"
    emotions_dict[emotions.disgust] = "disgusted"
    emotions_dict[emotions.anger] = "angry"
    emotions_dict[emotions.surprise] = "surprised"
    emotions_dict[emotions.fear] = "afraid"
    emotions_dict[emotions.happiness] = "happy"
    strongest_confidence = max(emotions_dict.keys())
    return emotions_dict[strongest_confidence]


class KnownPersonStorage():
    def __init__(self, name):
        self.data = {}

    def is_known(self, name):
        return name in self.data

    def check_execute(self, name):
        if name not in self.data:
            self.data[name] = rospy.get_time()
        current_time = rospy.get_time()
        if current_time - self.data[name] > 10:
            self.data[name] = current_time
            return True
        return False

class NewFaceDetector(object):
    def __init__(self):
        super(NewFaceDetector, self).__init__()
        rospy.init_node("new_face_detector")
        self.people_storage = KnownPersonStorage()
        self.speech_publisher = rospy.Publisher("hopper/cloud_say", String, queue_size=4)
        self.detection_subscriber = rospy.Subscriber("identified_faces", FaceDetectionImage, self.new_identified_image, queue_size=10)
        rospy.spin()

    def new_identified_image(self, identified_image):
        for face in identified_image.detected_faces:
            name = face.identified_name
            strong_emotion = get_strongest_emotion(face)
            response = ""
            if self.people_storage.is_known(name):
                response += choice(["Hello ", "Hi "])
            response += name + " "
            if self.people_storage.check_execute(name):
                response += "You seem " + strong_emotion + ". "
                if strong_emotion in ["angry", "afraid", "disgusted", "sad", "contempt"]:
                    response += choice(["Are you feeling okay?", "Would you like to talk about it?", "Can I help you somehow?"])
                if strong_emotion in ["happy"]:
                    response += choice(["What has you in such a good mood?"])
                if strong_emotion in ["surprised"]:
                    response += choice(["Did you just see something interesting?"])
                self.speech_publisher.publish(response)


if __name__ == "__main__":
    NewFaceDetector()
