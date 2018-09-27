#!/usr/bin/env python

from os.path import expanduser
from os import listdir
import os.path as path

import random
import rospy
import rospkg

import pygame.mixer as sound
from std_msgs.msg import String


class HopperSpeechHandles(object):
    def __init__(self):
        super(HopperSpeechHandles, self).__init__()
        rospy.init_node("hopper_speech_core", anonymous=True)
        sound.init()
        rospack = rospkg.RosPack()
        self.primary_sound_path = rospack.get_path('hopper_speech') + "/sounds/"
        self.secondary_sound_path = expanduser("~") + "/Music/"
        rospy.Subscriber("hopper_play_sound", String, self.on_play_sound)
        rospy.Subscriber("hopper/sound/play_random", String, self.on_play_random)
        rospy.spin()

    def on_play_sound(self, string):
        ogg_file_name = string.data + ".ogg"
        wav_file_name = string.data + ".wav"
        if path.isfile(self.primary_sound_path + ogg_file_name):
            self.play_file(self.primary_sound_path + ogg_file_name)
        elif path.isfile(self.primary_sound_path + wav_file_name):
            self.play_file(self.primary_sound_path + wav_file_name)
        elif path.isfile(self.secondary_sound_path + ogg_file_name):
            self.play_file(self.secondary_sound_path + ogg_file_name)
        elif path.isfile(self.secondary_sound_path + wav_file_name):
            self.play_file(self.secondary_sound_path + wav_file_name)

    def on_play_random(self, string_msg):
        folder_name = string_msg.data
        if not folder_name:
            folder_name = ""
        else:
            folder_name += "/"
        sound_file_names = filter(lambda name: ".wav" in name or ".ogg" in name, listdir(self.secondary_sound_path + folder_name))
        selected_file = random.choice(sound_file_names)
        self.play_file(self.secondary_sound_path + folder_name + selected_file)

    def play_file(self, path):
        file = sound.Sound(path)
        file.play()


if __name__ == '__main__':
    HopperSpeechHandles()
