from hexapod_gait_engine import GaitController
from hexapod_ik_driver import IkDriver, Vector2
from dynamixel_driver import DynamixelDriver, search_usb_2_ax_port
from time import sleep

print("Start")

servo_driver = DynamixelDriver("COM8")
ik_driver = IkDriver(servo_driver)
controller = GaitController(ik_driver)
print("Starting move")
controller.direction = Vector2(6, 0)
# controller.rotation = -10
sleep(10)
controller.rotation = 0
controller.direction = Vector2(0, 0)
sleep(5)

print("Finished")

controller.stop(True)
raw_input("Press enter to exit")
print("Exiting")
