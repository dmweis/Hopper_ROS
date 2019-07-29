#!/usr/bin/env python
from __future__ import division

import rospy
from math import radians

from std_srvs.srv import Empty
from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest,\
MoveLegsUntilCollision, MoveLegsUntilCollisionRequest, MoveCoreToPosition,\
MoveCoreToPositionRequest, MoveLegsToRelativePosition, MoveLegsToRelativePositionRequest,\
MoveBodyRelative, MoveBodyRelativeRequest
from geometry_msgs.msg import Vector3

from hexapod.hexapod_ik_driver import Vector2, Vector3, LegFlags


class BlindClimbController(object):
    def __init__(self):
        super(BlindClimbController, self).__init__()
        rospy.init_node("blind_climb_controller")
        self.default_frame_id = "body_link"
        rospy.wait_for_service("hopper/move_limbs_individual")
        rospy.wait_for_service("hopper/move_body_core")
        rospy.wait_for_service("hopper/move_legs_until_collision")
        rospy.wait_for_service("hopper/move_legs_to_relative_position")
        rospy.wait_for_service("hopper/move_body_relative")
        rospy.wait_for_service("hopper/step_to_relaxed")
        self.move_legs_service = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
        self.move_legs_until_collision_service = rospy.ServiceProxy("hopper/move_legs_until_collision", MoveLegsUntilCollision)
        self.move_body_core_service = rospy.ServiceProxy("hopper/move_body_core", MoveCoreToPosition)
        self.move_legs_relative = rospy.ServiceProxy("hopper/move_legs_to_relative_position", MoveLegsToRelativePosition)
        self.move_body_relative = rospy.ServiceProxy("hopper/move_body_relative", MoveBodyRelative)
        self.move_to_relaxed = rospy.ServiceProxy("/hopper/step_to_relaxed", Empty)

    def step_left(self, vector):
        # lift left
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = 0.05
        request.right_middle.z = 0.05
        request.left_rear.z = 0.05
        self.move_legs_relative(request)
        # move all
        request = MoveLegsToRelativePositionRequest()
        request.left_front = vector.to_vector3()
        request.right_front = -vector.to_vector3()
        request.left_middle = -vector.to_vector3()
        request.right_middle = vector.to_vector3()
        request.left_rear = vector.to_vector3()
        request.right_rear = -vector.to_vector3()
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = -0.05
        request.right_middle.z = -0.05
        request.left_rear.z = -0.05
        self.move_legs_relative(request)

    def step_right(self, vector):
        # lift left
        request = MoveLegsToRelativePositionRequest()
        request.right_front.z = 0.05
        request.left_middle.z = 0.05
        request.right_rear.z = 0.05
        self.move_legs_relative(request)
        # move all
        request = MoveLegsToRelativePositionRequest()
        request.left_front = -vector.to_vector3()
        request.right_front = vector.to_vector3()
        request.left_middle = vector.to_vector3()
        request.right_middle = -vector.to_vector3()
        request.left_rear = -vector.to_vector3()
        request.right_rear = vector.to_vector3()
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.right_front.z = -0.05
        request.left_middle.z = -0.05
        request.right_rear.z = -0.05
        self.move_legs_relative(request)

    def relax_legs(self):
        self.move_to_relaxed()

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

    def test_relative_move(self, obstacle_height):
        # rospy.sleep(5)
        # move middle legs forward
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = 0.05
        request.right_middle.z = 0.05
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.x = 0.07
        request.right_middle.x = 0.07
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = -0.05
        request.right_middle.z = -0.05
        self.move_legs_relative(request)
        # raise body
        request = MoveBodyRelativeRequest()
        request.translation.z = 0.05
        request.rotation.y = -radians(10)
        self.move_body_relative(request)
        # raise front legs
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = obstacle_height + 0.04
        request.left_front.y = 0.07
        request.right_front.z = obstacle_height + 0.04
        request.right_front.y = -0.07
        self.move_legs_relative(request)
        # move front legs forward
        request = MoveLegsToRelativePositionRequest()
        request.left_front.x = 0.15
        request.left_front.y = -0.07
        request.right_front.x = 0.15
        request.right_front.y = 0.07
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = -0.04
        request.right_front.z = -0.04
        self.move_legs_relative(request)
        # move center of mass forward
        request = MoveBodyRelativeRequest()
        request.translation.x = 0.14
        self.move_body_relative(request)
        # move hind legs
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = 0.03
        request.left_rear.x = 0.09
        request.right_rear.z = 0.03
        request.right_rear.x = 0.09
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = -0.03
        request.left_rear.x = 0.09
        request.right_rear.z = -0.03
        request.right_rear.x = 0.09
        self.move_legs_relative(request)
        # move middle legs
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = obstacle_height + 0.05
        request.left_middle.x = 0.14
        request.right_middle.z = obstacle_height + 0.05
        request.right_middle.x = 0.14
        self.move_legs_relative(request)
        request = MoveBodyRelativeRequest()
        request.translation.x = 0.08
        request.translation.z = 0.04
        request.rotation.y = radians(10)
        self.move_body_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.x = 0.06
        request.right_middle.x = 0.06
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = -0.06
        request.right_middle.z = -0.06
        self.move_legs_relative(request)
        # move front legs
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = 0.03
        request.left_front.x = 0.08
        request.right_front.z = 0.03
        request.right_front.x = 0.08
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_front.z = -0.03
        request.left_front.x = 0.08
        request.right_front.z = -0.03
        request.right_front.x = 0.08
        self.move_legs_relative(request)
        # try lift hind legs
        request = MoveBodyRelativeRequest()
        request.translation.x = 0.05
        self.move_body_relative(request)
        # left rear leg
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = 0.03
        request.left_rear.x = 0.05
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = -0.03
        request.left_rear.x = 0.05
        self.move_legs_relative(request)
        # right rear leg
        request = MoveLegsToRelativePositionRequest()
        request.right_rear.z = 0.03
        request.right_rear.x = 0.05
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.right_rear.z = -0.03
        request.right_rear.x = 0.05
        self.move_legs_relative(request)
        # move middle legs
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = 0.03
        request.left_middle.x = 0.03
        request.right_middle.z = 0.03
        request.right_middle.x = 0.03
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = -0.03
        request.left_middle.x = 0.03
        request.right_middle.z = -0.03
        request.right_middle.x = 0.03
        self.move_legs_relative(request)
        # lift hind legs on platform
        request = MoveBodyRelativeRequest()
        request.translation.x = 0.1
        self.move_body_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = obstacle_height + 0.04
        request.left_rear.x = 0.06
        request.right_rear.z = obstacle_height + 0.04
        request.right_rear.x = 0.06
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.x = 0.06
        request.right_rear.x = 0.06
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_rear.z = -0.05
        request.right_rear.z = -0.05
        self.move_legs_relative(request)
        # realign middle legs
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = 0.03
        request.left_middle.x = 0.03
        request.right_middle.z = 0.03
        request.right_middle.x = 0.03
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.left_middle.z = -0.03
        request.left_middle.x = 0.03
        request.right_middle.z = -0.03
        request.right_middle.x = 0.03
        self.move_legs_relative(request)

    def main_climb(self):
        self.move_body_core(Vector3(0, 0, 0.05))
        self.move_leg(MoveCoreToPositionRequest.LEFT_MIDDLE, "base_link", Vector3(0.05, 0.22, 0))
        self.move_leg_until_collision(MoveCoreToPositionRequest.LEFT_MIDDLE, "base_link", Vector3(0.05, 0.22, -0.2))


if __name__ == "__main__":
    controller = BlindClimbController()
    # controller.step_left(Vector2(0.02, 0))
    # controller.step_right(Vector2(-0.02, 0))
    controller.test_relative_move(0.05)
    controller.step_left(Vector2(0.01))
    controller.step_right(Vector2(0.01))
    controller.relax_legs()
