#!/usr/bin/env python

#!/usr/bin/env python

from __future__ import print_function

import json
import random
import os.path
import os.path as path
from os.path import expanduser, exists
from os import listdir, makedirs

import rospy
import rospkg
import pygame.mixer as sound
from std_msgs.msg import String
from google.cloud import texttospeech


class SpeechClient(object):
    def __init__(self):
        super(SpeechClient, self).__init__()
        self.client = texttospeech.TextToSpeechClient()
        self.voice = texttospeech.types.VoiceSelectionParams(
            language_code='en-US',
            ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)
        self.audio_config = texttospeech.types.AudioConfig(audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16)

    def synthesize(self, text, filepath):
        synthesis_input = texttospeech.types.SynthesisInput(text=text)
        response = self.client.synthesize_speech(synthesis_input, self.voice, self.audio_config)
        with open(filepath, 'wb') as out:
            out.write(response.audio_content)


class HopperCloudSpeech(object):
    def __init__(self):
        super(HopperCloudSpeech, self).__init__()
        rospy.init_node("hopper_cloud_speech", anonymous=True)
        sound.init(22050, -16, 1, 1024)
        self.caching_folder_path = expanduser("~") + "/Music/cloud_cache"
        if not exists(self.caching_folder_path):
            makedirs(self.caching_folder_path)
        self.cloud_client = SpeechClient()
        rospy.Subscriber("hopper/cloud_say", String, self.on_play_sound, queue_size=1)
        rospy.spin()

    def on_play_sound(self, string):
        text = string.data

        hashed_text = str(abs(hash(text)))

        filename = path.join(self.caching_folder_path, hashed_text + ".wav")

        if exists(filename):
            self.play_file(filename)
        else:
            self.cloud_client.synthesize(text, filename)
            self.play_file(filename)

    def play_file(self, path):
        file = sound.Sound(path)
        file.play()
        rospy.sleep(file.get_length())


if __name__ == '__main__':
    HopperCloudSpeech()
