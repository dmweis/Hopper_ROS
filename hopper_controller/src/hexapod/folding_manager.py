import rospy

from hopper_controller.msg import HexapodMotorPositions, LegMotorPositions
from .hexapod_ik_driver import LegFlags


def move_towards(target, current, step=0.1):
    if abs(target-current) < step:
        return target
    else:
        if target > current:
            return current + step
        else:
            return current - step


class FoldingManager(object):
    def __init__(self, body_controller):
        super(FoldingManager, self).__init__()
        self.body_controller = body_controller

    def unfold(self):
        self.body_controller.set_torque(False)
        while True:
            rospy.logerr(self.body_controller.read_hexapod_motor_positions())
