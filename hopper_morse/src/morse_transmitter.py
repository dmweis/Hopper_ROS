#!/usr/bin/env python

import rospy

from hopper_controller.msg import SingleLegCommand
from std_msgs.msg import String

MORSE = {'A': '.-',     'B': '-...',   'C': '-.-.', 
        'D': '-..',    'E': '.',      'F': '..-.',
        'G': '--.',    'H': '....',   'I': '..',
        'J': '.---',   'K': '-.-',    'L': '.-..',
        'M': '--',     'N': '-.',     'O': '---',
        'P': '.--.',   'Q': '--.-',   'R': '.-.',
        'S': '...',    'T': '-',      'U': '..-',
        'V': '...-',   'W': '.--',    'X': '-..-',
        'Y': '-.--',   'Z': '--..',

        '0': '-----',  '1': '.----',  '2': '..---',
        '3': '...--',  '4': '....-',  '5': '.....',
        '6': '-....',  '7': '--...',  '8': '---..',
        '9': '----.',

        ' ': ' '
        }

LEG_DOWN = SingleLegCommand()
LEG_DOWN.single_leg_mode_on = True
LEG_DOWN.selected_leg = SingleLegCommand.RIGHT_FRONT
LEG_DOWN.position.z = -0.025

LEG_UP = SingleLegCommand()
LEG_UP.single_leg_mode_on = True
LEG_UP.selected_leg = SingleLegCommand.RIGHT_FRONT
LEG_UP.position.z = -0.01


class MorseEncoder(object):
    def __init__(self):
        super(MorseEncoder, self).__init__()
        rospy.init_node('morse_encoder')
        self.leg_publisher = rospy.Publisher('hopper/single_leg_command', SingleLegCommand, queue_size=10)
        self.unit = 0.2
        self.hold_up(2)
        # rospy.Subscriber('hopper/morse', String, self.on_morse, queue_size=1)
        self.do_phrase("CQ")
        self.hold_up(0.1)

    def do_phrase(self, phrase):
        for character in map(lambda character : MORSE[character.upper()], phrase):
            self.do_character(character)
        self.hold_down(self.unit * 4)

    def do_character(self, character):
        print "Doing:", character
        if character == ' ':
            self.hold_up(self.unit * 7)
            return
        for symbol in character:
            self.do_symbol(symbol)
        self.hold_up(self.unit * 3)

    def do_symbol(self, symbol):
        if symbol == '.':
            self.hold_down(self.unit)
            self.hold_up(self.unit)
        elif symbol == '-':
            self.hold_down(self.unit * 3)
            self.hold_up(self.unit)

    def hold_down(self, time):
        self.leg_publisher.publish(LEG_DOWN)
        rospy.sleep(time)
        self.leg_publisher.publish(LEG_UP)

    def hold_up(self, time):
        self.leg_publisher.publish(LEG_UP)
        rospy.sleep(time)


if __name__ == "__main__":
    MorseEncoder()
