import rospy

from hopper_controller.msg import HexapodMotorPositions, LegMotorPositions
from .hexapod_ik_driver import LegFlags

a = 240
b = 60


def move_towards(target, current, step=0.4):
    if abs(target-current) < step:
        return target, True
    else:
        if target > current:
            return current + step, False
        else:
            return current - step, False


def move_leg(leg, coxa=None, femur=None, tibia=None):
    coxa_done = True
    femur_done = True
    tibia_done = True
    if coxa:
        leg.coxa, coxa_done = move_towards(coxa, leg.coxa)
    if femur:
        leg.femur, femur_done = move_towards(femur, leg.femur)
    if tibia:
        leg.tibia, tibia_done = move_towards(tibia, leg.tibia)
    return coxa_done and femur_done and tibia_done


class FoldingManager(object):
    def __init__(self, body_controller):
        super(FoldingManager, self).__init__()
        self.body_controller = body_controller
        self.last_motor_position = None

    def position_femur_tibia(self):
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        while True:
            rospy.sleep(0.01)
            lf = move_leg(self.last_motor_position.left_front, None, 60, 240)
            lm = move_leg(self.last_motor_position.left_middle, None, 60, 240)
            lr = move_leg(self.last_motor_position.left_rear, None, 60, 240)
            rf = move_leg(self.last_motor_position.right_front, None, 240, 60)
            rm = move_leg(self.last_motor_position.right_middle, None, 240, 60)
            rr = move_leg(self.last_motor_position.right_rear, None, 240, 60)
            self.body_controller.set_motors(self.last_motor_position)
            if lf and lm and lr and rf and rm and rr:
                break
        rospy.sleep(0.05)

    def unfold(self):
        self.position_femur_tibia()
        current_position = self.body_controller.read_hexapod_motor_positions()
        self.last_motor_position = current_position
        # left side
        left_middle_backwards = current_position.left_middle.coxa > 150.0
        right_middle_backwards = current_position.right_middle.coxa < 150.0
        while True:
            rospy.sleep(0.01)
            self.last_motor_position.left_middle.coxa, left_done = move_towards(150, self.last_motor_position.left_middle.coxa)
            self.last_motor_position.right_middle.coxa, right_done = move_towards(150, self.last_motor_position.right_middle.coxa)
            self.body_controller.set_motors(self.last_motor_position)
            if left_done and right_done:
                break
        while True:
            rospy.sleep(0.01)
            self.last_motor_position.left_front.coxa, one = move_towards(150, self.last_motor_position.left_front.coxa)
            self.last_motor_position.right_front.coxa, two = move_towards(150, self.last_motor_position.right_front.coxa)

            self.last_motor_position.left_rear.coxa, three = move_towards(150, self.last_motor_position.left_rear.coxa)
            self.last_motor_position.right_rear.coxa, four = move_towards(150, self.last_motor_position.right_rear.coxa)
            self.body_controller.set_motors(self.last_motor_position)
            if one and two and three and four:
                break
        rospy.sleep(0.05)
        self.body_controller.set_torque(False)
        while True:
            rospy.logerr(self.body_controller.read_hexapod_motor_positions())

