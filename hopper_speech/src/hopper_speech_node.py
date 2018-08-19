#!/usr/bin/env python

from os.path import expanduser
import os.path as path
import rospy
import rospkg
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String


class HopperSpeechHandles(object):
    def __init__(self):
        super(HopperSpeechHandles, self).__init__()
        rospy.init_node("hopper_speech_core", anonymous=True)
        rospack = rospkg.RosPack()
        self.primary_sound_path = rospack.get_path('hopper_speech') + "/sounds/"
        self.secondary_sound_path = expanduser("~") + "/sounds/"
        self.sound_client = SoundClient()
        self.play_sound_sub = rospy.Subscriber("hopper_play_sound", String, self.on_play_sound)
        rospy.spin()

    def on_play_sound(self, string):
        ogg_file_name = string.data + ".ogg"
        wav_file_name = string.data + ".wav"
        if path.isfile(self.primary_sound_path + ogg_file_name):
            self.sound_client.playWave(self.primary_sound_path + ogg_file_name)
        elif path.isfile(self.primary_sound_path + wav_file_name):
            self.sound_client.playWave(self.primary_sound_path + wav_file_name)
        elif path.isfile(self.secondary_sound_path + ogg_file_name):
            self.sound_client.playWave(self.secondary_sound_path + ogg_file_name)
        elif path.isfile(self.secondary_sound_path + wav_file_name):
            self.sound_client.playWave(self.secondary_sound_path + wav_file_name)


if __name__ == '__main__':
    HopperSpeechHandles()
