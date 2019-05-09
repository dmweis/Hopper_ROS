#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import tf.transformations as transformations

from math import degrees
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class AbductionDetector(object):
    tilt_limit = 40
    speech_time_limit = 5

    def __init__(self):
        super(AbductionDetector, self).__init__()
        rospy.init_node("hopper_abduction_detector")
        self.is_upside_down = False
        self.last_speech_trigger = 0
        self.speech_publisher = rospy.Publisher("hopper_play_sound", String, queue_size=2)
        self.move_pub = rospy.Publisher('hopper_schedule_move', String, queue_size=2)
        rospy.Subscriber("/hopper/imu/data", Imu, self.on_imu_msg, queue_size=10)
        rospy.spin()

    def on_imu_msg(self, imu_msg):
        quaternion = imu_msg.orientation
        euler = transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        if abs(degrees(-euler[0])) < self.tilt_limit and abs(degrees(euler[1])) < self.tilt_limit:
            self.on_upside_down()
        else:
            self.on_right_side_up()

    def on_upside_down(self):
        self.move_pub.publish("happy_spin")
        if self.is_upside_down:
            return
        else:
            if rospy.get_time() - self.last_speech_trigger > self.speech_time_limit:
                self.speech_publisher.publish("take_your_paws")
                self.last_speech_trigger = rospy.get_time()
        self.is_upside_down = True

    def on_right_side_up(self):
        if self.is_upside_down:
            self.move_pub.publish("cancel")
        self.is_upside_down = False


if __name__ == "__main__":
    AbductionDetector()
