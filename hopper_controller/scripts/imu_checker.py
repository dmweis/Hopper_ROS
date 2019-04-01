#!/usr/bin/env python

from __future__ import print_function

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from tf import transformations


class ImuChecker(object):
    def __init__(self):
        super(ImuChecker, self).__init__()
        rospy.init_node("IMU_checker")
        self.initial_orientation = None
        rospy.Subscriber("hopper/Imu/data", Imu, self.on_imu_msg, queue_size=10)
        rospy.spin()

    def on_imu_msg(self, msg):
        if not self.initial_orientation:
            self.initial_orientation = msg.orientation
            return
        current_orientation = msg.orientation - self.initial_orientation
        print(transformations.euler_from_quaternion(current_orientation))


if __name__ == "__main__":
    ImuChecker()
