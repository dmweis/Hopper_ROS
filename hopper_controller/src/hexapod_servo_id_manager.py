#!/usr/bin/env python

from __future__ import print_function
from dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port

servo_driver = DynamixelDriver(search_usb_2_ax_port())

print("Starting search")
servo_ids = servo_driver.search_servos()
print("Found {0} servos".format(len(servo_ids)))

legs_to_assign = {
    1: "Left Front Coxa",
    3: "Left Front Femur",
    5: "Left Front Tibia",
    2: "Right Front Coxa",
    4: "Right Front Femur",
    6: "Right Front Tibia",
    13: "Left Middle Coxa",
    15: "Left Middle Femur",
    17: "Left Middle Tibia",
    14: "Right Middle Coxa",
    16: "Right Middle Femur",
    18: "Right Middle Tibia",
    7: "Left Rear Coxa",
    9: "Left Rear Femur",
    11: "Left Rear Tibia",
    8: "Right Rear Coxa",
    10: "Right Rear Femur",
    12: "Right Rear Tibia"
}

print("Setting ids up to make sure none are taken")
first_free_id = max(legs_to_assign.keys())
for servo_id in servo_ids:
    if servo_id >= first_free_id:
        continue
    free_id = -1
    for new_id in range(first_free_id, 253):
        if not servo_driver.ping(new_id):
            free_id = new_id
            break
    servo_driver.set_id(servo_id, free_id)

servo_ids = servo_driver.search_servos()

for servo_id in servo_ids:
    servo_driver.set_led(servo_id, False)

for servo_id in servo_ids:
    servo_driver.set_led(servo_id, True)
    print("Servo {0} led turned on".format(servo_id))
    for key, value in legs_to_assign.items():
        print("{0}: {1}".format(key, value))
    selected_position = int(raw_input("Enter role of servo with LED on"))
    servo_driver.set_id(servo_id, selected_position)
    del legs_to_assign[selected_position]
    servo_driver.set_led(selected_position, False)

print ("All servos correctly assigned")
