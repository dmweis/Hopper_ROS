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


def on_telemetrics(telemetrics):
    print(telemetrics)


controller.telemetrics_callback = on_telemetrics

print("Finished")
raw_input("Press enter to exit")
controller.stop()
print("Exiting")
