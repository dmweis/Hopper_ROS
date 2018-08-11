#!/usr/bin/env python

from __future__ import print_function
from src.dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port

servo_driver = DynamixelDriver(search_usb_2_ax_port())

temporary_id = 50
switches = [
    (1, 8),
    (2, 7),
    (3, 10),
    (4, 9),
    (5, 12),
    (6, 11),
    (14, 13),
    (16, 15),
    (18, 17)
]

def swap_ids(from_id, to_id):
    servo_driver.set_id(from_id, temporary_id)
    servo_driver.set_id(to_id, from_id)
    servo_driver.set_id(temporary_id, to_id)

print("Starting search")
servo_ids = servo_driver.search_servos(end_id=20)
print("Found {0} servos".format(len(servo_ids)))
for from_id, to_id in switches:
    swap_ids(from_id, to_id)

print("finished swapping ids")
servo_driver.close()
