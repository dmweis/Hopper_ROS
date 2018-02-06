from __future__ import print_function
from __future__ import print_function
from hexapod_gait_engine import GaitController
from hexapod_ik_driver import IkDriver, Vector2, Vector3
from dynamixel_driver import DynamixelDriver, search_usb_2_ax_port
from time import sleep

print("Start")

servo_driver = DynamixelDriver("COM8")
ik_driver = IkDriver(servo_driver)
controller = GaitController(ik_driver)
while not controller.ready:
    print("waiting...")
    sleep(1)
print("Starting move")

print("Changing")
controller.update_relaxed_position(transform=Vector3(0, 0, 5))
sleep(2)
controller.direction = Vector2(3, 0)
sleep(5)
controller.direction = Vector2(0, 0)

print("Finished")

controller.stop()
print("Exiting")
