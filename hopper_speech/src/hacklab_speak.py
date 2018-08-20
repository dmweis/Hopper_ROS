#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String

class HacklabSpeaker(object):
    def __init__(self):
        super(HacklabSpeaker, self).__init__()
        rospy.init_node("Hacklab_speaker_node")
        self.mqtt = mqtt.Client()
        #self.mqtt.on_connect = self.on_connect
        self.mqtt.connect("mqtt.hacklab")
        rospy.Subscriber("hacklab/say", String, self.on_say, queue_size=1)
        rospy.Subscriber("hacklab/play", String, self.on_play, queue_size=1)
        rospy.spin()

    def on_connect(self, client, userdata, flags, rc):
        pass
        #client.subscribe()

    def on_say(self, msg):
        message = msg.data
        self.mqtt.publish("sound/g1/speak", message)

    def on_play(self, msg):
        file_name = msg.data
        self.mqtt.publish("sound/g1/play", file_name)


if __name__ == "__main__":
    HacklabSpeaker()
