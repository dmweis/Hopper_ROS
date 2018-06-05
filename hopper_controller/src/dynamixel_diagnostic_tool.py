#!/usr/bin/env python

from __future__ import print_function
from dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port

servo_driver = DynamixelDriver(search_usb_2_ax_port())
print("Starting search")
servo_ids = servo_driver.search_servos()
print("Found {0} servos".format(len(servo_ids)))

for servo_id in servo_ids:
    limits = servo_driver.read_angle_limits(servo_id)
    print("For servo {0} limits are {1} (min, max)".format(servo_id, limits))
    servo_driver.set_ccw_max_angle_limit(servo_id, 1023)
    servo_driver.set_cw_min_angle_limit(servo_id, 0)
    limits = servo_driver.read_angle_limits(servo_id)
    print("After fix: servo {0} limits are {1} (min, max)".format(servo_id, limits))

servo_driver.close()
print("Driver closed\nBye!")
