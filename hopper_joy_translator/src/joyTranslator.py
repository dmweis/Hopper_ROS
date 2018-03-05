#!/usr/bin/env python

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
        new_message.linear.x = joy.axes[0] * 6
        new_message.linear.y = joy.axes[1] * 6
        new_message.angular.x = joy.axes[2] * 10
        self.pub.publish(new_message)


if __name__ == '__main__':
    JoyTranslator()
