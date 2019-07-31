#!/usr/bin/env python
from __future__ import print_function

import rospy
import tf2_ros

from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest
from geometry_msgs.msg import Vector3, Pose

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

legs = [
    MoveLegsToPositionRequest.LEFT_FRONT,
    MoveLegsToPositionRequest.RIGHT_FRONT,
    MoveLegsToPositionRequest.LEFT_MIDDLE,
    MoveLegsToPositionRequest.RIGHT_MIDDLE,
    MoveLegsToPositionRequest.LEFT_REAR,
    MoveLegsToPositionRequest.RIGHT_REAR
]

leg_id_to_name = {
    MoveLegsToPositionRequest.LEFT_FRONT : "left front foot",
    MoveLegsToPositionRequest.RIGHT_FRONT : "right front foot",
    MoveLegsToPositionRequest.LEFT_MIDDLE : "left middle foot",
    MoveLegsToPositionRequest.RIGHT_MIDDLE : "right middle foot",
    MoveLegsToPositionRequest.LEFT_REAR : "left rear foot",
    MoveLegsToPositionRequest.RIGHT_REAR : "right rear foot"
}

leg_id_to_frame = {
    MoveLegsToPositionRequest.LEFT_FRONT : "left_front_button",
    MoveLegsToPositionRequest.RIGHT_FRONT : "right_front_button",
    MoveLegsToPositionRequest.LEFT_MIDDLE : "left_middle_button",
    MoveLegsToPositionRequest.RIGHT_MIDDLE : "right_middle_button",
    MoveLegsToPositionRequest.LEFT_REAR : "left_rear_button",
    MoveLegsToPositionRequest.RIGHT_REAR : "right_rear_button"
}

def make_6_dof_marker(frame_id, name, position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = name
    int_marker.description = name + " foot marker"
    int_marker.pose.position = position
    int_marker.scale = 0.2
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.03
    box_marker.scale.y = 0.03
    box_marker.scale.z = 0.03
    box_marker.color.r = 0.5
    box_marker.color.g = 0.0
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.interaction_mode = InteractiveMarkerControl.BUTTON
    box_control.markers.append( box_marker )
    int_marker.controls.append( box_control )

    x_move_control = InteractiveMarkerControl()
    x_move_control.name = "move_x"
    x_move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    x_move_control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(x_move_control)

    y_move_control = InteractiveMarkerControl()
    y_move_control.orientation.y = 1
    y_move_control.orientation.w = 1
    y_move_control.name = "move_y"
    y_move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    y_move_control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(y_move_control)

    z_move_control = InteractiveMarkerControl()
    z_move_control.orientation.z = 1
    z_move_control.orientation.w = 1
    z_move_control.name = "move_z"
    z_move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    z_move_control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(z_move_control)

    return int_marker


class InteractiveLegMove(object):
    def __init__(self):
        super(InteractiveLegMove, self).__init__()
        rospy.init_node("interactive_leg_move")
        rospy.wait_for_service("hopper/move_limbs_individual")
        self.move_legs = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
        self.server = InteractiveMarkerServer("hopper_feet_marker_server")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        position = None
        for leg in legs:
            while not rospy.is_shutdown():
                try:
                    self.add_leg(leg)
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass
        rospy.spin()

    def add_leg(self, leg_id):
        position = self.get_position_for(leg_id_to_frame[leg_id])
        marker = make_6_dof_marker("base_link", leg_id_to_name[leg_id], position)
        def marker_feedback(msg):
            if msg.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
                position = msg.pose.position
                self.move_to_point(position, leg_id)
        self.server.insert(marker, marker_feedback)
        self.server.applyChanges()

    def move_to_point(self, position, leg_id):
        request = MoveLegsToPositionRequest()
        request.header.frame_id = "base_link"
        request.selected_legs = leg_id
        point = Vector3()
        point.x = position.x
        point.y = position.y
        point.z = position.z
        request.leg_positions.left_front = point
        request.leg_positions.right_front = point
        request.leg_positions.left_middle = point
        request.leg_positions.right_middle = point
        request.leg_positions.left_rear = point
        request.leg_positions.right_rear = point
        self.move_legs(request)

    def get_position_for(self, link):
        return self.tf_buffer.lookup_transform("base_link", link, rospy.Time()).transform.translation


if __name__ == "__main__":
    InteractiveLegMove()
