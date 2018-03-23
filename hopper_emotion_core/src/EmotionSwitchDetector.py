#!/usr/bin/env python

import rospy
from hopper_emotion_core.msg import FaceDetectionImage, EmotionChange

def get_strongest_emotion(detected_face):
    emotions = detected_face.face_attributes.emotions
    emotions_dict = {}
    emotions_dict[emotions.sadness] = "sadness"
    emotions_dict[emotions.neutral] = "neutral"
    emotions_dict[emotions.contempt] = "contempt"
    emotions_dict[emotions.disgust] = "disgust"
    emotions_dict[emotions.anger] = "anger"
    emotions_dict[emotions.surprise] = "surprise"
    emotions_dict[emotions.fear] = "fear"
    emotions_dict[emotions.happiness] = "happiness"
    strongest_confidence = max(emotions_dict.keys())
    return emotions_dict[strongest_confidence]

class EmotionChangeDetector(object):
    def __init__(self):
        rospy.init_node("Emotional_switch_detector")
        self.face_dictionary = {}
        self.emotion_publisher = rospy.Publisher("emotional_change", EmotionChange, queue_size=4)
        self.detection_subscriber = rospy.Subscriber("identified_faces", FaceDetectionImage, self.new_identified_image, queue_size=10)
        rospy.spin()

    def new_identified_image(self, identified_image):
        for face in identified_image.detected_faces:
            if not face.identified_name:
                continue
            strong_emotion = get_strongest_emotion(face)
            if face.identified_name not in self.face_dictionary:
                self.face_dictionary[face.identified_name] = strong_emotion
                continue
            if strong_emotion != self.face_dictionary[face.identified_name]:
                emotion_change = EmotionChange()
                emotion_change.person_name = face.identified_name
                emotion_change.previous_emotion = self.face_dictionary[face.identified_name]
                emotion_change.new_emotion = strong_emotion
                self.emotion_publisher.publish(emotion_change)
                self.face_dictionary[face.identified_name] = strong_emotion

if __name__ == "__main__":
    EmotionChangeDetector()
