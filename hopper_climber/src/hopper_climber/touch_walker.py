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


class TouchWalker(object):
    def __init__(self):
        super(TouchWalker, self).__init__()
        rospy.init_node("blind_climb_controller")
        self.default_frame_id = "body_link"
        rospy.wait_for_service("hopper/move_legs_to_relative_position")
        rospy.wait_for_service("hopper/move_legs_to_relative_position_until_hit")
        rospy.wait_for_service("hopper/move_body_relative")
        rospy.wait_for_service("hopper/step_to_relaxed")
        self.move_legs_relative = rospy.ServiceProxy("hopper/move_legs_to_relative_position", MoveLegsToRelativePosition)
        self.move_legs_relative_until_hit = rospy.ServiceProxy("hopper/move_legs_to_relative_position_until_hit", MoveLegsToRelativePosition)
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
        self.move_legs_relative_until_hit(request)

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
        self.move_legs_relative_until_hit(request)

    def relax_legs(self):
        self.move_to_relaxed()


if __name__ == "__main__":
    controller = TouchWalker()
    controller.step_left(Vector2(0.02, 0))
    controller.step_right(Vector2(-0.02, 0))
    controller.relax_legs()
