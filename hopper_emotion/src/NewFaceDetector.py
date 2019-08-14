#!/usr/bin/env python

import rospy
from hopper_emotion.msg import FaceDetectionImage
from std_msgs.msg import String

def get_strongest_emotion(detected_face):
    emotions = detected_face.face_attributes.emotions
    emotions_dict = {}
    emotions_dict[emotions.sadness] = "sad"
    emotions_dict[emotions.neutral] = "indifferent"
    emotions_dict[emotions.contempt] = "contempt"
    emotions_dict[emotions.disgust] = "disgusted"
    emotions_dict[emotions.anger] = "angry"
    emotions_dict[emotions.surprise] = "surprised"
    emotions_dict[emotions.fear] = "afraid"
    emotions_dict[emotions.happiness] = "happy"
    strongest_confidence = max(emotions_dict.keys())
    return emotions_dict[strongest_confidence]

class NewFaceDetector(object):
    def __init__(self):
        super(NewFaceDetector, self).__init__()
        rospy.init_node("new_face_detector")
        self.face_dictionary = {}
        self.speech_publisher = rospy.Publisher("hopper/cloud_say", String, queue_size=4)
        self.detection_subscriber = rospy.Subscriber("identified_faces", FaceDetectionImage, self.new_identified_image, queue_size=10)
        rospy.spin()

    def new_identified_image(self, identified_image):
        for face in identified_image.detected_faces:
            name = face.identified_name
            if not name:
                strong_emotion = get_strongest_emotion(face)
                if strong_emotion in ["angry", "afraid", "disgusted", "sad"]:
                    self.speech_publisher.publish("Hi! You seem " + strong_emotion + ". Are you okay?")
                else:
                    self.speech_publisher.publish("Hello! You seem " + strong_emotion)
            elif name not in self.face_dictionary.keys():
                self.face_dictionary[name] = rospy.get_time()
                strong_emotion = get_strongest_emotion(face)
                self.speech_publisher.publish("Hello " + name + ". Nice to see you! You seem " + strong_emotion)
            elif rospy.get_time() - self.face_dictionary[name] > 10:
                self.face_dictionary[name] = rospy.get_time()
                strong_emotion = get_strongest_emotion(face)
                if strong_emotion in ["angry", "afraid", "disgusted", "sad"]:
                    self.speech_publisher.publish(name + " you seem " + strong_emotion + ". Are you okay?")
                else:
                    self.speech_publisher.publish(name + " you seem " + strong_emotion)


if __name__ == "__main__":
    NewFaceDetector()
