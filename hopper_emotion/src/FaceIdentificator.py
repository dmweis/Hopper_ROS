#!/usr/bin/env python

from io import BytesIO
from time import time
import rospy
import cognitive_face
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from hopper_emotion_core.msg import DetectedFace, FaceRectangle, FaceAttributes, Emotions, FaceDetectionImage, NameAndPersonId
from hopper_emotion_core.srv import GetNameByPersonId


# disable warnings on unsecure requests done by the MS face api library
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)


class FaceIdentificator(object):
    def __init__(self):
        self.__last_update = time()
        rospy.init_node("Emotioanl_core", anonymous=True)
        api_key = rospy.get_param("face_api_key")
        api_url = rospy.get_param("face_api_base_url")
        cognitive_face.Key.set(api_key)
        cognitive_face.BaseUrl.set(api_url)
        rospy.wait_for_service("get_name_by_person_id")
        self.get_name_by_person_id_database = rospy.ServiceProxy("get_name_by_person_id", GetNameByPersonId)
        self.image_subscriber = rospy.Subscriber("detected_faces", FaceDetectionImage, self.new_person_callback, queue_size=4)
        self.image_publisher = rospy.Publisher("identified_faces", FaceDetectionImage, queue_size=1)
        self.save_new_person = rospy.Publisher("hopper_register_new_person", NameAndPersonId, queue_size=10)
        rospy.spin()

    def _get_name_from_person_id(self, person_id):
        database_name = self.get_name_by_person_id_database(person_id)
        if not database_name.name:
            name = cognitive_face.person.get("primary_user_group", person_id)['name']
            rospy.logwarn("Creating new user for " + str(name))
            new_person = NameAndPersonId()
            new_person.person_id = person_id
            new_person.name = name
            self.save_new_person.publish(new_person)
            return name
        else:
            return database_name.name

    def new_person_callback(self, face_detection_image):
        face_ids = [face.face_id for face in face_detection_image.detected_faces]
        if len(face_ids) > 0:
            result = cognitive_face.face.identify(face_ids, "primary_user_group")
            for identified in result:
                if len(identified['candidates']) > 0:
                    name = self._get_name_from_person_id(str(identified['candidates'][0]['personId']))
                    for face in face_detection_image.detected_faces:
                        if face.face_id == identified['faceId']:
                            face.identified_name = name
        self.image_publisher.publish(face_detection_image)

if __name__ == "__main__":
    FaceIdentificator()
