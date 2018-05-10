#!/usr/bin/env python

from __future__ import division
from threading import Thread
import math
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics, WalkingMode, HopperMoveCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import tf.transformations as transformations
import tf2_ros


from hexapod.hexapod_gait_engine import GaitEngine, MovementController, TripodGait
from hexapod.hexapod_ik_driver import IkDriver, Vector2, Vector3
from dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port


class SoundPlayer(object):
    def __init__(self, speech_publisher):
        super(SoundPlayer, self).__init__()
        self.publisher = speech_publisher

    def say(self, file_name):
        self.publisher.publish(file_name)


class MessagePublisher(Thread):
    def __init__(self):
        super(MessagePublisher, self).__init__()
        self._callbacks = []
        self._rate = rospy.Rate(30)
        self.start()

    def add_message_sender(self, callback):
        self._callbacks.append(callback)

    def run(self):
        while not rospy.is_shutdown():
            for callback in self._callbacks:
                callback()
            self._rate.sleep()


class JointStatePublisher(object):
    def __init__(self, publisher):
        super(JointStatePublisher, self).__init__()
        self._publisher = publisher
        self._last_message = JointState()

    def update_joint_states(self, joint_names, joint_positions):
        """
        :param joint_names: list of names of the joints
        :param joint_positions: positions of joints
        :return:
        """
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joint_positions
        joint_state.velocity = []
        joint_state.effort = []
        self._last_message = joint_state

    def publish(self):
        self._last_message.header.stamp = rospy.Time.now()
        self._publisher.publish(self._last_message)


class OdomPublisher(object):
    def __init__(self, transform_broadcaster, parent_link_name="odom", child_link_name="base_footprint"):
        super(OdomPublisher, self).__init__()
        self._transform_broadcaster = transform_broadcaster
        self._parent_link_name = parent_link_name
        self._child_link_name = child_link_name
        # initialize default tf transform
        self._last_odometry_message = TransformStamped()
        self._last_odometry_message.header.frame_id = self._parent_link_name
        self._last_odometry_message.child_frame_id = self._child_link_name
        self.odometry_rotation = transformations.quaternion_from_euler(0, 0, 0)
        self._last_odometry_message.transform.rotation.x = self.odometry_rotation[0]
        self._last_odometry_message.transform.rotation.y = self.odometry_rotation[1]
        self._last_odometry_message.transform.rotation.z = self.odometry_rotation[2]
        self._last_odometry_message.transform.rotation.w = self.odometry_rotation[3]
        self.odometry_position = Vector2()

    def update_translation(self, direction, rotation):
        """
        :type direction: Vector2
        :type rotation: float
        """
        new_rotation = transformations.quaternion_from_euler(0, 0, math.radians(rotation))
        self.odometry_rotation = transformations.quaternion_multiply(new_rotation, self.odometry_rotation)
        current_rotation = transformations.euler_from_quaternion(self.odometry_rotation)[2]
        self.odometry_position += direction.rotate_by_angle_rad(current_rotation)
        message = TransformStamped()
        message.header.frame_id = self._parent_link_name
        message.child_frame_id = self._child_link_name
        message.transform.translation.x = self.odometry_position.x / 100
        message.transform.translation.y = self.odometry_position.y / 100
        message.transform.translation.z = 0
        message.transform.rotation.x = self.odometry_rotation[0]
        message.transform.rotation.y = self.odometry_rotation[1]
        message.transform.rotation.z = self.odometry_rotation[2]
        message.transform.rotation.w = self.odometry_rotation[3]
        self._last_odometry_message = message

    def publish(self):
        self._last_odometry_message.header.stamp = rospy.Time.now()
        self._transform_broadcaster.sendTransform(self._last_odometry_message)


class HeightPublisher(object):
    def __init__(self, transform_broadcaster, parent_link_name="base_footprint", child_link_name="base_stabilized"):
        super(HeightPublisher, self).__init__()
        self._transform_broadcaster = transform_broadcaster
        self._parent_link_name = parent_link_name
        self._child_link_name = child_link_name
        # initialize default tf transform
        self._last_message = TransformStamped()
        noraml_quaternion = transformations.quaternion_from_euler(0, 0, 0)
        self._last_message.transform.rotation.x = noraml_quaternion[0]
        self._last_message.transform.rotation.y = noraml_quaternion[1]
        self._last_message.transform.rotation.z = noraml_quaternion[2]
        self._last_message.transform.rotation.w = noraml_quaternion[3]
        self._last_message.header.frame_id = self._parent_link_name
        self._last_message.child_frame_id = self._child_link_name

    def update_height(self, height):
        self._last_message.transform.translation.z = height / 100

    def publish(self):
        self._last_message.header.stamp = rospy.Time.now()
        self._transform_broadcaster.sendTransform(self._last_message)


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        servo_driver = DynamixelDriver(search_usb_2_ax_port())
        self.join_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        # publisher for tf and joint states
        transform_broadcaster = tf2_ros.TransformBroadcaster()
        self._message_publisher = MessagePublisher()
        transform_publisher = OdomPublisher(transform_broadcaster)
        height_publisher = HeightPublisher(transform_broadcaster)
        joint_state_publisher = JointStatePublisher(self.join_state_publisher)
        self._message_publisher.add_message_sender(transform_publisher.publish)
        self._message_publisher.add_message_sender(joint_state_publisher.publish)
        self._message_publisher.add_message_sender(height_publisher.publish)
        # build controller
        ik_driver = IkDriver(servo_driver, joint_state_publisher)
        tripod_gait = TripodGait(ik_driver, height_publisher)
        gait_engine = GaitEngine(tripod_gait, transform_publisher)
        self.speech_publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)
        self.sound_player = SoundPlayer(self.speech_publisher)
        self.controller = MovementController(gait_engine, self.sound_player)
        self.telemetrics_publisher = rospy.Publisher('hopper_telemetrics', HexapodTelemetrics, queue_size=5)
        rospy.Subscriber("hopper_move_command", Twist, self.update_direction)
        rospy.Subscriber("hopper/move_command", HopperMoveCommand, self.on_move_command)
        rospy.Subscriber("hopper_walking_mode", WalkingMode, self.set_walking_mode)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        self.controller.subscribe_to_telemetrics(self.publish_telemetrics_data)

    def on_move_command(self, move_command):
        # convert directions from meter to cm
        direction = Vector2(move_command.direction.linear.x, move_command.direction.linear.y) * 100
        rotation = move_command.direction.angular.x
        self.controller.set_move_command(direction,
                                         rotation,
                                         move_command.lift_height,
                                         move_command.static_speed_mode,
                                         move_command.turbo)

    def set_walking_mode(self, walking_mode):
        static_speed_mode_enabled = walking_mode.selectedMode == WalkingMode.STATIC_SPEED
        self.controller.set_walking_mode(static_speed_mode_enabled, walking_mode.liftHeight)

    def update_direction(self, twist):
        direction = Vector2(twist.linear.x, twist.linear.y)
        rotation = twist.angular.x
        self.controller.set_direction(direction, rotation)

    def update_pose(self, twist):
        transform = Vector3(twist.linear.x, twist.linear.y, twist.linear.z)
        rotation = Vector3(twist.angular.x, twist.angular.y, twist.angular.z)
        self.controller.set_relaxed_pose(transform, rotation)

    def schedule_move(self, move_name):
        self.controller.schedule_move(move_name.data)

    def publish_telemetrics_data(self, telemetrics):
        msg = HexapodTelemetrics()
        msg.servos = []
        for id, voltage, temperature in telemetrics:
            tele = ServoTelemetrics()
            tele.id = id
            tele.temperature = temperature
            tele.voltage = voltage
            msg.servos.append(tele)
        self.telemetrics_publisher.publish(msg)

    def spin(self):
        rospy.spin()
        self.controller.stop()


if __name__ == '__main__':
    HexapodController().spin()
