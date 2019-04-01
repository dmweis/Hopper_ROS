#!/usr/bin/env python

from __future__ import print_function

import rospy
import numpy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from tf import transformations


quat_msg_to_array = lambda q: numpy.array([q.x, q.y, q.z, q.w])


class ImuChecker(object):
    def __init__(self):
        super(ImuChecker, self).__init__()
        rospy.init_node("IMU_checker")
        self.initial_orientation = None
        rospy.Subscriber("hopper/imu/data", Imu, self.on_imu_msg, queue_size=10)
        rospy.spin()

    def on_imu_msg(self, msg):
        if not self.initial_orientation:
            self.initial_orientation = msg.orientation
            return
        current_orientation = transformations.quaternion_multiply(
            quat_msg_to_array(msg.orientation),
            transformations.quaternion_inverse(quat_msg_to_array(self.initial_orientation)))
        print(transformations.euler_from_quaternion(current_orientation))


if __name__ == "__main__":
    ImuChecker()
