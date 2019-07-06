#!/usr/bin/env python

from __future__ import print_function, division

import serial
import rospy

from hopper_feet_sensors.msg import FeetSensorData


class FeetSensor(object):
    def __init__(self):
        super(FeetSensor, self).__init__()
        rospy.init_node("feet_sensors")
        self.feet_publisher = rospy.Publisher("hopper/feet", FeetSensorData, queue_size=10)
        with serial.Serial('/dev/hopper_feet', 9600) as port:
            port.timeout = 1
            while not rospy.is_shutdown():
                data = port.readline().strip().split(' ')
                if len(data) == 6:
                    new_msg = FeetSensorData()
                    new_msg.front_left = bool(int(data[5]))
                    new_msg.front_right = bool(int(data[0]))
                    new_msg.middle_left = bool(int(data[3]))
                    new_msg.middle_right = bool(int(data[2]))
                    new_msg.rear_left = bool(int(data[4]))
                    new_msg.rear_right = bool(int(data[1]))
                    self.feet_publisher.publish(new_msg)
                else:
                    rospy.logdebug("String format error")


if __name__ == "__main__":
    FeetSensor()
