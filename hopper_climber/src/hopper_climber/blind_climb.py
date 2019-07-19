#!/usr/bin/env python

import rospy

from enum import IntEnum
from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest, MoveLegsUntilCollision, MoveLegsUntilCollisionRequest, MoveCoreToPosition, MoveCoreToPositionRequest
from geometry_msgs.msg import Vector3


class LegFlags(IntEnum):
    NONE = 0
    LEFT_FRONT = 1
    RIGHT_FRONT = 2
    LEFT_REAR = 4
    RIGHT_REAR = 8
    LEFT_MIDDLE = 16
    RIGHT_MIDDLE = 32
    ALL = LEFT_FRONT | RIGHT_FRONT | LEFT_REAR | RIGHT_REAR | LEFT_MIDDLE | RIGHT_MIDDLE
    FRONT = LEFT_FRONT | RIGHT_FRONT
    REAR = LEFT_REAR | RIGHT_REAR
    MIDDLE = LEFT_MIDDLE | RIGHT_MIDDLE
    RIGHT = RIGHT_FRONT | RIGHT_REAR | RIGHT_MIDDLE
    LEFT = LEFT_FRONT | LEFT_REAR | LEFT_MIDDLE
    RF_LR_CROSS = RIGHT_FRONT | LEFT_REAR
    LF_RR_CROSS = LEFT_FRONT | RIGHT_REAR
    LEFT_TRIPOD = LEFT_FRONT | RIGHT_MIDDLE | LEFT_REAR
    RIGHT_TRIPOD = RIGHT_FRONT | LEFT_MIDDLE | RIGHT_REAR


class BlindClimbController(object):
    def __init__(self):
        super(BlindClimbController, self).__init__()
        rospy.init_node("blind_climb_controller")
        self.default_frame_id = "body_link"
        rospy.wait_for_service("hopper/move_limbs_individual")
        rospy.wait_for_service("hopper/move_body_core")
        rospy.wait_for_service("hopper/move_legs_until_collision")
        self.move_legs_service = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
        self.move_legs_until_collision_service = rospy.ServiceProxy("hopper/move_legs_until_collision", MoveLegsUntilCollision)
        self.move_body_core_service = rospy.ServiceProxy("hopper/move_body_core", MoveCoreToPosition)

    def move_leg(self, leg_id, frame_id, vector):
        request = MoveLegsToPositionRequest()
        request.header.frame_id = frame_id
        request.selected_legs = leg_id
        request.left_front = vector
        request.left_middle = vector
        request.left_rear = vector
        request.right_front = vector
        request.right_middle = vector
        request.right_rear = vector
        self.move_legs_service(request)

    def move_leg_until_collision(self, leg_id, frame_id, vector):
        request = MoveLegsUntilCollisionRequest()
        request.header.frame_id = frame_id
        request.selected_legs = leg_id
        request.left_front = vector
        request.left_middle = vector
        request.left_rear = vector
        request.right_front = vector
        request.right_middle = vector
        request.right_rear = vector
        self.move_legs_until_collision_service(request)

    def move_body_core(self, vector):
        request = MoveCoreToPositionRequest()
        request.header.frame_id = self.default_frame_id
        request.used_legs = LegFlags.ALL
        request.core_movement = vector
        self.move_body_core_service(request)

    def main_climb(self):
        self.move_body_core(Vector3(0, 0, 0.05))
        self.move_leg(MoveCoreToPositionRequest.LEFT_MIDDLE, "base_link", Vector3(0.05, 0.22, 0))
        self.move_leg_until_collision(MoveCoreToPositionRequest.LEFT_MIDDLE, "base_link", Vector3(0.05, 0.22, -0.2))


if __name__ == "__main__":
    controller = BlindClimbController()
    controller.main_climb()
    # rospy.init_node("leg_move_tester")
    # rospy.wait_for_service("hopper/move_limbs_individual")
    # move_legs = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
    # move_legs_until_collision = rospy.ServiceProxy("hopper/move_legs_until_collision", MoveLegsUntilCollision)
    # move_core = rospy.ServiceProxy("hopper/move_body_core", MoveCoreToPosition)

    
    # until_request = MoveLegsUntilCollisionRequest()
    # until_request.header.frame_id = "body_link"
    # until_request.selected_legs = MoveLegsUntilCollisionRequest.LEFT_MIDDLE
    # until_request.left_middle = Vector3(0, 0.25, -0.2)
    # move_legs_until_collision(until_request)

    # # core move
    # body_request = MoveCoreToPositionRequest()
    # body_request.header.frame_id = "body_link"
    # body_request.used_legs = 63
    # body_request.core_movement = Vector3(0.05, 0, 0)
    # move_core(body_request)
