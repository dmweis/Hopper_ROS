from __future__ import division

import rospy
import tf2_ros

from threading import Event
from Queue import Queue, Empty
from hopper_controller.srv import MoveLegsToPosition, MoveCoreToPosition, MoveLegsUntilCollision
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker
from hexapod.hexapod_ik_driver import LegPositions, Vector3, LegFlags
from hopper_feet_sensors.msg import FeetSensorData
from pyquaternion import Quaternion


class LegController(object):
    def __init__(self, gait_engine):
        super(LegController, self).__init__()
        self.gait_engine = gait_engine
        self.motion_queue = Queue()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.last_feet_msg = FeetSensorData()

        self.marker_publisher = rospy.Publisher("leg_move_marker", Marker, queue_size=10)
        # Subscribers
        rospy.Subscriber("hopper/feet", FeetSensorData, self.on_feet_msg, queue_size=10)

        rospy.Service('hopper/move_limbs_individual', MoveLegsToPosition, self.move_legs)
        rospy.Service('hopper/move_body_core', MoveCoreToPosition, self.move_body)
        rospy.Service('hopper/move_legs_until_collision', MoveLegsUntilCollision, self.move_until_hit)
        rospy.Service('hopper/move_to_relaxed', Empty, self.move_to_relaxed)

    def on_feet_msg(self, feet_msg):
        self.last_feet_msg = feet_msg

    def move_legs(self, move_legs_cmd):
        local_frame = "base_link"
        command_frame = move_legs_cmd.header.frame_id
        ros_transform = self.tf_buffer.lookup_transform(local_frame, command_frame, rospy.Time()).transform
        frame_translation_ros, frame_rotation_ros = ros_transform.translation, ros_transform.rotation
        frame_rotation = Quaternion(frame_rotation_ros.w, frame_rotation_ros.x, frame_rotation_ros.y, frame_rotation_ros.z)
        frame_translation = Vector3.ros_vector3_to_overload_vector(frame_translation_ros)
        move_legs_overloaded = LegPositions.ros_leg_positions_to_leg_positions(move_legs_cmd)
        new_positions = LegPositions(
             (move_legs_overloaded.left_front * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_front * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.left_middle * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_middle * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.left_rear * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_rear * frame_rotation + frame_translation) * 100.0
        )
        current_positions = self.gait_engine.get_current_leg_positions()
        desired_position = current_positions.update_from_other(new_positions, LegFlags(move_legs_cmd.selected_legs))
        task_finished_event = Event()
        self.motion_queue.put((task_finished_event, desired_position))
        # debug marker
        # self.display_marker(desired_position.left_front.x / 100, desired_position.left_front.y / 100, desired_position.left_front.z / 100)
        task_finished_event.wait()
        return True

    def move_body(self, move_legs_cmd):
        local_frame = "base_link"
        command_frame = move_legs_cmd.header.frame_id
        ros_transform = self.tf_buffer.lookup_transform(local_frame, command_frame, rospy.Time()).transform
        frame_translation_ros, frame_rotation_ros = ros_transform.translation, ros_transform.rotation
        frame_rotation = Quaternion(frame_rotation_ros.w, frame_rotation_ros.x, frame_rotation_ros.y, frame_rotation_ros.z)
        frame_translation = Vector3.ros_vector3_to_overload_vector(frame_translation_ros)
        move_vector_overload = (-Vector3.ros_vector3_to_overload_vector(move_legs_cmd.core_movement) * frame_rotation + frame_translation) * 100.0
        current_positions = self.gait_engine.get_current_leg_positions()
        new_positions = current_positions.transform(move_vector_overload, LegFlags(move_legs_cmd.used_legs))
        task_finished_event = Event()
        self.motion_queue.put((task_finished_event, new_positions))
        task_finished_event.wait()
        return True

    def move_until_hit(self, move_legs_cmd):
        local_frame = "base_link"
        command_frame = move_legs_cmd.header.frame_id
        ros_transform = self.tf_buffer.lookup_transform(local_frame, command_frame, rospy.Time()).transform
        frame_translation_ros, frame_rotation_ros = ros_transform.translation, ros_transform.rotation
        frame_rotation = Quaternion(frame_rotation_ros.w, frame_rotation_ros.x, frame_rotation_ros.y, frame_rotation_ros.z)
        frame_translation = Vector3.ros_vector3_to_overload_vector(frame_translation_ros)
        move_legs_overloaded = LegPositions.ros_leg_positions_to_leg_positions(move_legs_cmd)
        new_positions = LegPositions(
             (move_legs_overloaded.left_front * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_front * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.left_middle * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_middle * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.left_rear * frame_rotation + frame_translation) * 100.0
            ,(move_legs_overloaded.right_rear * frame_rotation + frame_translation) * 100.0
        )
        current_positions = self.gait_engine.get_current_leg_positions()
        desired_position = current_positions.update_from_other(new_positions, LegFlags(move_legs_cmd.selected_legs))
        # move check loop
        move_done = False
        move_dist = 0.5 # distance to move with each step in cm
        colliding_legs = LegFlags.NONE
        midstep_positions = current_positions.clone()
        while not move_done:
            still_moving = False
            if not self.last_feet_msg.left_front:
                still_moving = still_moving or midstep_positions.left_front.move_towards_at_speed(desired_position.left_front, move_dist)
            else:
                colliding_legs |= LegFlags.LEFT_FRONT
            if not self.last_feet_msg.right_front:
                still_moving = still_moving or midstep_positions.right_front.move_towards_at_speed(desired_position.right_front, move_dist)
            else:
                colliding_legs |= LegFlags.RIGHT_FRONT
            if not self.last_feet_msg.left_middle:
                still_moving = still_moving or midstep_positions.left_middle.move_towards_at_speed(desired_position.left_middle, move_dist)
            else:
                colliding_legs |= LegFlags.LEFT_MIDDLE
            if not self.last_feet_msg.right_middle:
                still_moving = still_moving or midstep_positions.right_middle.move_towards_at_speed(desired_position.right_middle, move_dist)
            else:
                colliding_legs |= LegFlags.RIGHT_MIDDLE
            if not self.last_feet_msg.left_rear:
                still_moving = still_moving or midstep_positions.left_rear.move_towards_at_speed(desired_position.left_rear, move_dist)
            else:
                colliding_legs |= LegFlags.LEFT_REAR
            if not self.last_feet_msg.right_rear:
                still_moving = still_moving or midstep_positions.right_rear.move_towards_at_speed(desired_position.right_rear, move_dist)
            else:
                colliding_legs |= LegFlags.RIGHT_REAR
            if still_moving:
                task_finished_event = Event()
                self.motion_queue.put((task_finished_event, midstep_positions))
                task_finished_event.wait()
            else:
                move_done = True
        return int(colliding_legs), midstep_positions

    def move_to_relaxed(self, srvs_request):
        relaxed_pose = self.gait_engine.get_relaxed_pose()
        task_finished_event = Event()
        self.motion_queue.put((task_finished_event, relaxed_pose))
        task_finished_event.wait()
        return EmptyResponse()

    def execute_motion(self):
        try:
            event, motion = self.motion_queue.get_nowait()
            self.gait_engine.move_to_new_pose(motion, 22)
            event.set()
        except Empty:
            rospy.logerr("Motion queue was empty")

    def is_motion_queued(self):
        return not self.motion_queue.empty()

    def display_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        self.marker_publisher.publish(marker)
