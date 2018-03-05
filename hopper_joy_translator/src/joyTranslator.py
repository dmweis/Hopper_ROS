#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTranslator(object):
    def __init__(self):
        rospy.init_node('hopper_joy_trnalstor')
        self.sub = rospy.Subscriber("joy", Joy, self.translate_message)
        self.pub = rospy.Publisher("hopper_move_command", Twist, queue_size=10)
        rospy.spin()

    def translate_message(self, joy):
        new_message = Twist()
        x = 0
        y = 0
        rot = 0
        if math.sqrt(joy.axes[1] * joy.axes[1] + joy.axes[0] * joy.axes[0]) > 0.2:
            x = joy.axes[1] * 6
            y = joy.axes[0] * 6
        if abs(joy.axes[3]) > 0.2:
            rot = joy.axes[3] * 10
        new_message.linear.x = x
        new_message.linear.y = y
        new_message.angular.x = rot
        self.pub.publish(new_message)


if __name__ == '__main__':
    JoyTranslator()
