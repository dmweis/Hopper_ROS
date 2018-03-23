#!/usr/bin/env python

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


def translate_result(image, query_result):
    result_image = FaceDetectionImage()
    result_image.image = image
    result_image.detected_faces = []
    for result in query_result:
        face = DetectedFace()
        face.face_id = result['faceId']
        # Rectangle
        face.face_rectangle = FaceRectangle()
        face.face_rectangle.width = result['faceRectangle']['width']
        face.face_rectangle.height = result['faceRectangle']['height']
        face.face_rectangle.top = result['faceRectangle']['top']
        face.face_rectangle.left = result['faceRectangle']['left']
        # Emotions
        emotions = Emotions()
        emo = result['faceAttributes']['emotion']
        emotions.sadness = emo["sadness"]
        emotions.neutral = emo["neutral"]
        emotions.contempt = emo["contempt"]
        emotions.disgust = emo["disgust"]
        emotions.anger = emo["anger"]
        emotions.surprise = emo["surprise"]
        emotions.fear = emo["fear"]
        emotions.happiness = emo["happiness"]
        # faceAttributes
        face.face_attributes = FaceAttributes()
        face.face_attributes.emotions = emotions
        face.face_attributes.gender = result['faceAttributes']['gender']
        face.face_attributes.age = result['faceAttributes']['age']
        result_image.detected_faces.append(face)
    return result_image


class EmotionalCore(object):
    def __init__(self):
        self.__last_update = time()
        rospy.init_node("Emotioanl_core", anonymous=True)
        api_key = rospy.get_param("face_api_key")
        api_url = rospy.get_param("face_api_base_url")
        cognitive_face.Key.set(api_key)
        cognitive_face.BaseUrl.set(api_url)
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.new_image_callback, queue_size=4)
        self.json_publisher = rospy.Publisher('person_stream', String, queue_size=1)
        self.face_detection_publisher = rospy.Publisher('detected_faces', FaceDetectionImage, queue_size=1)
        rospy.spin()

    def new_image_callback(self, compressed_image):
        if time() - self.__last_update > 1:
            self.__last_update = time()
            with BytesIO(compressed_image.data) as image_stream:
                result = cognitive_face.face.detect(image_stream, attributes='age,gender,emotion')
                self.json_publisher.publish(str(result))
                self.face_detection_publisher.publish(translate_result(compressed_image, result))


if __name__ == "__main__":
    EmotionalCore()
