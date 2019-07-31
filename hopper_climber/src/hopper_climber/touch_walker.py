#!/usr/bin/env python
from __future__ import division

import rospy
from math import radians

from std_srvs.srv import Empty
from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest,\
MoveLegsUntilCollision, MoveLegsUntilCollisionRequest, MoveCoreToPosition,\
MoveCoreToPositionRequest, MoveLegsToRelativePosition, MoveLegsToRelativePositionRequest,\
MoveBodyRelative, MoveBodyRelativeRequest, \
ReadCurrentLegPositions, ReadCurrentLegPositionsRequest
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
        rospy.wait_for_service("hopper/read_current_leg_positions")
        self.move_legs_relative = rospy.ServiceProxy("hopper/move_legs_to_relative_position", MoveLegsToRelativePosition)
        self.move_legs_relative_until_hit = rospy.ServiceProxy("hopper/move_legs_to_relative_position_until_hit", MoveLegsToRelativePosition)
        self.move_body_relative = rospy.ServiceProxy("hopper/move_body_relative", MoveBodyRelative)
        self.move_to_relaxed = rospy.ServiceProxy("/hopper/step_to_relaxed", Empty)
        self.read_current_positions = rospy.ServiceProxy("hopper/read_current_leg_positions", ReadCurrentLegPositions)

    def read_legs(self, frame_id):
        request = ReadCurrentLegPositionsRequest()
        request.header.frame_id = "base_footprint"
        return self.read_current_positions(request)

    def step_left(self, vector, lift_height=0.05):
        current_positions = self.read_legs("base_footprint")
        # lift left
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.left_front.z = lift_height - current_positions.left_front.z
        request.leg_positions.right_middle.z = lift_height - current_positions.right_middle.z
        request.leg_positions.left_rear.z = lift_height - current_positions.left_rear.z
        self.move_legs_relative(request)
        # move all
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.left_front = vector.to_vector3()
        request.leg_positions.right_front = -vector.to_vector3()
        request.leg_positions.left_middle = -vector.to_vector3()
        request.leg_positions.right_middle = vector.to_vector3()
        request.leg_positions.left_rear = vector.to_vector3()
        request.leg_positions.right_rear = -vector.to_vector3()
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.left_front.z = -lift_height
        request.leg_positions.right_middle.z = -lift_height
        request.leg_positions.left_rear.z = -lift_height
        self.move_legs_relative_until_hit(request)

    def step_right(self, vector, lift_height):
        current_positions = self.read_legs("base_footprint")
        # lift left
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.right_front.z = lift_height - current_positions.right_front.z
        request.leg_positions.left_middle.z = lift_height - current_positions.left_middle.z
        request.leg_positions.right_rear.z = lift_height - current_positions.right_rear.z
        self.move_legs_relative(request)
        # move all
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.left_front = -vector.to_vector3()
        request.leg_positions.right_front = vector.to_vector3()
        request.leg_positions.left_middle = vector.to_vector3()
        request.leg_positions.right_middle = -vector.to_vector3()
        request.leg_positions.left_rear = -vector.to_vector3()
        request.leg_positions.right_rear = vector.to_vector3()
        self.move_legs_relative(request)
        request = MoveLegsToRelativePositionRequest()
        request.leg_positions.right_front.z = -lift_height
        request.leg_positions.left_middle.z = -lift_height
        request.leg_positions.right_rear.z = -lift_height
        self.move_legs_relative_until_hit(request)

    def relax_legs(self):
        self.move_to_relaxed()


if __name__ == "__main__":
    controller = TouchWalker()
    controller.step_left(Vector2(0.03, 0), 0.1)
    controller.step_right(Vector2(0.03, 0), 0.1)
