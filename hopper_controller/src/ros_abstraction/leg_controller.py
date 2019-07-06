from __future__ import division

import rospy
import tf2_ros

from threading import Event
from Queue import Queue, Empty
from hopper_controller.srv import MoveLegsToPosition, MoveCoreToPosition
from hexapod.hexapod_ik_driver import LegPositions, Vector3, LegFlags


class LegController(object):
    def __init__(self, gait_engine):
        super(LegController, self).__init__()
        self.gait_engine = gait_engine
        self.motion_queue = Queue()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Service('hopper/move_limbs_individual', MoveLegsToPosition, self.move_legs)
        rospy.Service('hopper/move_body_core', MoveCoreToPosition, self.move_body)

    def move_legs(self, move_legs_cmd):
        local_frame = "base_link"
        command_frame = move_legs_cmd.header.frame_id
        frame_transform = Vector3.ros_vector3_to_overload_vector(self.tf_buffer.lookup_transform(local_frame, command_frame, rospy.Time()).transform.translation)
        move_legs_overloaded = LegPositions.ros_leg_positions_to_leg_positions(move_legs_cmd)
        new_positions = LegPositions(
             (move_legs_overloaded.left_front + frame_transform) * 100.0
            ,(move_legs_overloaded.right_front + frame_transform) * 100.0
            ,(move_legs_overloaded.left_middle + frame_transform) * 100.0
            ,(move_legs_overloaded.right_middle + frame_transform) * 100.0
            ,(move_legs_overloaded.left_rear + frame_transform) * 100.0
            ,(move_legs_overloaded.right_rear + frame_transform) * 100.0
        )
        current_positions = self.gait_engine.get_current_leg_positions()
        desired_position = current_positions.update_from_other(new_positions, LegFlags(move_legs_cmd.selected_legs))
        task_finished_event = Event()
        self.motion_queue.put((task_finished_event, desired_position))
        task_finished_event.wait()
        return True

    def move_body(self, move_legs_cmd):
        local_frame = "base_link"
        command_frame = move_legs_cmd.header.frame_id
        frame_transform = Vector3.ros_vector3_to_overload_vector(self.tf_buffer.lookup_transform(local_frame, command_frame, rospy.Time()).transform.translation)
        move_legs_overloaded = LegPositions.ros_leg_positions_to_leg_positions(move_legs_cmd)
        new_positions = LegPositions(
             (move_legs_overloaded.left_front + frame_transform) * 100.0
            ,(move_legs_overloaded.right_front + frame_transform) * 100.0
            ,(move_legs_overloaded.left_middle + frame_transform) * 100.0
            ,(move_legs_overloaded.right_middle + frame_transform) * 100.0
            ,(move_legs_overloaded.left_rear + frame_transform) * 100.0
            ,(move_legs_overloaded.right_rear + frame_transform) * 100.0
        )
        current_positions = self.gait_engine.get_current_leg_positions()
        desired_position = current_positions.update_from_other(new_positions, LegFlags(move_legs_cmd.selected_legs))
        task_finished_event = Event()
        self.motion_queue.put((task_finished_event, desired_position))
        task_finished_event.wait()
        return True

    def execute_motion(self):
        try:
            event, motion = self.motion_queue.get_nowait()
            self.gait_engine.move_to_new_pose(motion, 22)
            event.set()
        except Empty:
            rospy.logerr("Motion queue was empty")

    def is_motion_queued(self):
        return not self.motion_queue.empty()
