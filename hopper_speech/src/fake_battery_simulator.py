#!/usr/bin/env python

from __future__ import division

import rospy

from random import randint
from hopper_msgs.msg import ServoTelemetry, HexapodTelemetry

if __name__ == "__main__":
    rospy.init_node("hopper_fake_telemetry")
    publisher = rospy.Publisher("hopper_telemetry", HexapodTelemetry, queue_size=20)
    while not rospy.is_shutdown():
        rospy.sleep(1)
        voltage = 10.5 + randint(0, 200) / 100
        hexapod_telemetry = HexapodTelemetry()
        for servo_id in range(18):
            telemetry = ServoTelemetry()
            telemetry.id = servo_id
            telemetry.temperature = 45
            telemetry.voltage = voltage
            hexapod_telemetry.servos.append(telemetry)
        publisher.publish(hexapod_telemetry)