from quadruped_gait_engine import GaitController
from quadruped_ik_driver import IkDriver, Vector2
from src.dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port
from time import sleep

print("Start")

servo_driver = DynamixelDriver(search_usb_2_ax_port())
ik_driver = IkDriver(servo_driver)
controller = GaitController(ik_driver)
print("Starting move")
controller.direction = Vector2(3, 0)
controller.rotation = -10
sleep(2)
controller.rotation = 0
controller.direction = Vector2(0, 0)
print("Finished")

controller.stop()
input("Press enter to exit")
