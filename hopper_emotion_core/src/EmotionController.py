#!/usr/bin/env

from io import BytesIO
from time import time
import rospy
import cognitive_face
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

class EmotionalCore(object):
    def __init__(self):
        self.__last_update = time()
        rospy.init_node("Emotioanl_core", anonymous=True)
        api_key = rospy.get_param("face_api_key")
        api_url = rospy.get_param("face_api_base_url")
        cognitive_face.Key.set(api_key)
        cognitive_face.BaseUrl.set(api_url)
        self.image_subscriber = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.new_image_callback, queue_size=1)
        self.json_publisher = rospy.Publisher('person_stream', String)
        rospy.spin()

    def new_image_callback(self, compressed_image):
        if time() - self.__last_update > 3:
            self.__last_update = time()
            with BytesIO(compressed_image.data) as image_stream:
                result = cognitive_face.face.detect(image_stream, attributes='age,gender,emotion')
                self.json_publisher.publish(str(result))


if __name__ == "__main__":
    EmotionalCore()
