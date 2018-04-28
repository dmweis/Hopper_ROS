#!/usr/bin/env python

import rospy
import rospkg
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


class HopperSpeechHandles(object):
    def __init__(self):
        super(HopperSpeechHandles, self).__init__()
        rospy.init_node("hopper_speech_core", anonymous=True)
        self.sound_client = SoundClient()
        rospack = rospkg.RosPack()
        self.sound_path = rospack.get_path('hopper_speech') + "/sounds/"
        self.play_sound_sub = rospy.Subscriber("hopper_play_sound", String, self.on_play_sound)
        rospy.spin()

    def on_play_sound(self, string):
        self.sound_client.playWave(self.sound_path + string.data + ".ogg")


if __name__ == '__main__':
    HopperSpeechHandles()
