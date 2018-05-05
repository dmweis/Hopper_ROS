#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from steamcontroller import SteamController, SCButtons
from steamcontroller.events import EventMapper, Pos


class SteamControllerRosHandler(object):
    def __init__(self):
        super(SteamControllerRosHandler, self).__init__()
        rospy.init_node("steam_controller_node")
        evm = self._init_even_mapper()
        sc = SteamController(evm.process)
        sc.run()

    def _init_even_mapper(self):
        evm = EventMapper()
        evm.setButtonCallback(SCButtons.A, self.button_pressed_callback)
        evm.setButtonCallback(SCButtons.B, self.button_pressed_callback)
        evm.setButtonCallback(SCButtons.X, self.button_pressed_callback)
        evm.setButtonCallback(SCButtons.Y, self.button_pressed_callback)
        return evm

    def button_pressed_callback(self, evm, btn, pressed):
        rospy.logerr("Button {} was {}.".format(btn, 'pressed' if pressed else 'released'))

if __name__ == "__main__":
    SteamControllerRosHandler()