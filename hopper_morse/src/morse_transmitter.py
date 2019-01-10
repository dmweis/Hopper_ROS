#!/usr/bin/env python

import sys
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
LEG_DOWN.fast_mode = True
LEG_DOWN.single_leg_mode_on = True
LEG_DOWN.selected_leg = SingleLegCommand.LEFT_FRONT
LEG_DOWN.position.z = -0.02

LEG_UP = SingleLegCommand()
LEG_UP.fast_mode = True
LEG_UP.single_leg_mode_on = True
LEG_UP.selected_leg = SingleLegCommand.LEFT_FRONT
LEG_UP.position.z = 0

NOT_SINGLE_LEG_MODE = SingleLegCommand()
NOT_SINGLE_LEG_MODE.single_leg_mode_on = False


class MorseEncoder(object):
    def __init__(self):
        super(MorseEncoder, self).__init__()
        rospy.init_node('morse_encoder')
        self.leg_publisher = rospy.Publisher('hopper/single_leg_command', SingleLegCommand, queue_size=10)
        self.unit = 0.1
        self.hold_up(1)
        #rospy.Subscriber('hopper/morse', String, self.on_morse, queue_size=1)
        if len(sys.argv) < 2:
            print "No argument"
            exit()
        try:
            self.do_phrase(sys.argv[1])
        except:
            pass
        self.hold_up(0.1)

    def on_morse_command(self, msg):
        self.hold_up(1)
        self.do_phrase(msg.data)
        self.hold_up(0.5)
        self.lower_leg()

    def do_phrase(self, phrase):
        for character in map(lambda character : MORSE[character.upper()], phrase):
            self.do_character(character)
        self.hold_up(self.unit * 4)

    def do_character(self, character):
        print "Sending:", character
        if character == ' ':
            self.hold_up(self.unit * 7)
            return
        for symbol in character:
            self.do_symbol(symbol)
        self.hold_up(self.unit * 3)

    def do_symbol(self, symbol):
        if rospy.is_shutdown():
            raise Exception("Interrupted")
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

    def lower_leg(self):
        self.leg_publisher.publish(NOT_SINGLE_LEG_MODE)

if __name__ == "__main__":
    MorseEncoder()
