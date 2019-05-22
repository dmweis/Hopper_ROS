#!/usr/bin/env python

import rospy
import random

from ttastromech import TTAstromech
from std_msgs.msg import String


class R2D2Speech(object):
    def __init__(self):
        super(R2D2Speech, self).__init__()
        self.speaker = TTAstromech()
        rospy.init_node("hopper_r2d2_speech_core", anonymous=True)
        rospy.Subscriber("hopper_play_sound", String, self.on_play_sound)
        rospy.Subscriber("hopper/sound/play_random", String, self.on_play_sound)
        rospy.spin()

    def on_play_sound(self, string):
        self.speaker.speak(string.data.lower()[:8])


if __name__ == '__main__':
    R2D2Speech()
