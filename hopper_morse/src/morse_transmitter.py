#!/usr/bin/env python

import rospy

from hopper_controller.msg import SingleLegCommand
from std_msgs.msg import String

MORSE ={
    "A" : ".-",
    "B" : "-...",
    "C" : "-.-.",
    "D" : "-..",
    "E" : ".",
    "F" : "..-.",
    "G" : "--.",
    "H" : "....",
    "I" : "..",
    "J" : ".---",
    "K" : "-.-",
    "L" : ".-..",
    "M" : "--",
    "N" : "-.",
    "O" : "---",
    "P" : ".--.",
    "Q" : "--.-",
    "R" : ".-.",
    "S" : "...",
    "T" : "-",
    "U" : "..-",
    "V" : "...-",
    "W" : ".--",
    "X" : "-..-",
    "Y" : "-.--",
    "Z" : "--.."
    }

LEG_DOWN = SingleLegCommand()
LEG_DOWN.single_leg_mode_on = self.single_leg_mode_on
LEG_DOWN.selected_leg = SingleLegCommand.RIGHT_FRONT
LEG_DOWN.position.z = -0.2

LEG_UP = SingleLegCommand()
LEG_UP.single_leg_mode_on = self.single_leg_mode_on
LEG_UP.selected_leg = SingleLegCommand.RIGHT_FRONT
LEG_DOWN.position.z = 0


class MorseEncoder(object):
    def __init__(self):
        super(MorseEncoder, self).__init__()
        rospy.init_node('morse_encoder')
        self.leg_publisher = rospy.Publisher('hopper/single_leg_command', SingleLegCommand, queue_size=10)
        self.unit = 0.5
        # rospy.Subscriber('hopper/morse', String, self.on_morse, queue_size=1)
        self.do_word(map(lambda x : x.upper(x), "Hello world"))

    def do_word(self, word):
        for character in map(lambda character : MORSE[character], word):
            self.do_character(character)
        self.hold_down(self.unit * 4)

    def do_character(self, character):
        for symbol in character:
            self.do_symbol(symbol)
        self.hold_up(self.unit * 2)

    def do_symbol(self, symbol):
        if symbol == '.':
            self.hold_down(self.unit)
            self.hold_up(self.unit)
        elif symbol == '-':
            self.hold_down(self.unit * 3)
            self.hold_up(self.unit)

    def hold_down(self, time):
        self.leg_publisher(LEG_DOWN)
        rospy.sleep(time)
        self.leg_publisher(LEG_UP)

    def hold_up(self, time):
        rospy.sleep(time)


if __name__ == "__name__":
    MorseEncoder()
