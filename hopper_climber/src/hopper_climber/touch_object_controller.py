#!/usr/bin/env python

import rospy

from enum import IntEnum
from geometry_msgs.msg import PointStamped, Vector3
from std_msgs.msg import Int32
from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest, MoveLegsUntilCollision, MoveLegsUntilCollisionRequest, MoveCoreToPosition, MoveCoreToPositionRequest


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


class TouchPointController(object):
    def __init__(self):
        super(TouchPointController, self).__init__()
        rospy.init_node("touch_point_controller")
        self.selected_foot = LegFlags.LEFT_FRONT
        # services
        rospy.wait_for_service("hopper/move_limbs_individual")
        rospy.wait_for_service("hopper/move_body_core")
        rospy.wait_for_service("hopper/move_legs_until_collision")
        self.move_legs = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
        self.move_legs_until_collision = rospy.ServiceProxy("hopper/move_legs_until_collision", MoveLegsUntilCollision)
        self.move_body_core_service = rospy.ServiceProxy("hopper/move_body_core", MoveCoreToPosition)
        rospy.Subscriber("hopper/touch_point_controller/select_foot", Int32, self.on_select_foot_msg, queue_size=10)
        rospy.Subscriber("clicked_point", PointStamped, self.on_clicked_point, queue_size=1)
        rospy.spin()

    def on_clicked_point(self, point_msg):
        rospy.logdebug("Clicked point " + str(point_msg))
        request = MoveLegsToPositionRequest()
        request.header.frame_id = point_msg.header.frame_id
        request.selected_legs = int(self.selected_foot)
        point = Vector3()
        point.x = point_msg.point.x
        point.y = point_msg.point.y
        point.z = point_msg.point.z
        request.left_front = point
        request.right_front = point
        request.left_middle = point
        request.right_middle = point
        request.left_rear = point
        request.right_rear = point
        self.move_legs(request)

    def on_select_foot_msg(self, msg):
        self.selected_foot = LegFlags(msg.data)


if __name__ == "__main__":
    TouchPointController()
