from time import sleep
from dynamixel_driver import DynamixelDriver

driver = DynamixelDriver("COM8")

print("Hi")
driver.set_torque(1, False)
driver.set_torque(2, False)

driver.set_moving_speed(2, 100)
clockwise = True
driver.set_goal_position(2, 150)

load_limit = 10

def reverse_direction():
    global clockwise
    clockwise = not clockwise
    if  clockwise:
        driver.set_goal_position(2, 150)
    else:
        driver.set_goal_position(2, 900)

# driver.set_moving_speed(1, 1023)
# driver.set_goal_position(1, 150)
# sleep(2)
# driver.set_moving_speed(1, 100)
# driver.set_goal_position(1, 900)

while True:
    load = driver.read_present_load(2)
    if  clockwise and load > load_limit:
        print("reversed because load")
        reverse_direction()
        continue
    if not clockwise and load < -load_limit:
        print("reversed because load")
        reverse_direction()
        continue
    current_position = driver.read_current_position(2)
    if clockwise and current_position < 200:
        print("reversed")
        reverse_direction()
        continue
    if not clockwise and current_position > 850:
        print("reversed")
        reverse_direction()
        continue
    sleep(0.2)

# input("Press enter to exit")
driver.close()
