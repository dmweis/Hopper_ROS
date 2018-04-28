#!/usr/bin/env python

from threading import Thread
import math
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics, WalkingMode
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


class TransformPublisher(Thread):
    def __init__(self, join_state_publisher, transform_brodcaster):
        super(TransformPublisher, self).__init__()
        self._join_state_publisher = join_state_publisher
        self._transform_brodcaster = transform_brodcaster
        self._last_joint_state = JointState()
        self._last_odometry_message = TransformStamped()
        self.odometry_rotation = transformations.quaternion_from_euler(0, 0, 0)
        self.odometry_position = Vector2()

        self.rate = rospy.Rate(30)
        self.start()

    def update_translation(self, direction, rotation):
        """
        :type direction: Vector2
        :type rotation: float
        """
        new_rotation = transformations.quaternion_from_euler(0, 0, math.radians(rotation))
        self.odometry_rotation = transformations.quaternion_multiply(new_rotation, self.odometry_rotation)
        self.odometry_position += direction
        message = TransformStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.child_frame_id = "base_link"
        message.transform.translation.x = self.odometry_position.x / 100
        message.transform.translation.y = self.odometry_position.y / 100
        message.transform.translation.z = 0
        message.transform.rotation.x = self.odometry_rotation[0]
        message.transform.rotation.y = self.odometry_rotation[1]
        message.transform.rotation.z = self.odometry_rotation[2]
        message.transform.rotation.w = self.odometry_rotation[3]
        self._last_odometry_message = message

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
        self._last_joint_state = joint_state

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._last_joint_state.header.stamp = now
            self._last_odometry_message.header.stamp = now
            self._join_state_publisher.publish(self._last_joint_state)
            self._transform_brodcaster.sendTransform(self._last_odometry_message)
            self.rate.sleep()


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        servo_driver = DynamixelDriver(search_usb_2_ax_port())
        self.join_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        # publisher for tf and joint states
        transform_broadcaster = tf2_ros.TransformBroadcaster()
        transform_publisher = TransformPublisher(self.join_state_publisher, transform_broadcaster)
        # build controller
        ik_driver = IkDriver(servo_driver, transform_publisher)
        tripod_gait = TripodGait(ik_driver)
        gait_engine = GaitEngine(tripod_gait, transform_publisher)
        self.speech_publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)
        self.sound_player = SoundPlayer(self.speech_publisher)
        self.controller = MovementController(gait_engine, self.sound_player)
        self.telemetrics_publisher = rospy.Publisher('hopper_telemetrics', HexapodTelemetrics, queue_size=5)
        rospy.Subscriber("hopper_move_command", Twist, self.update_direction)
        rospy.Subscriber("hopper_walking_mode", WalkingMode, self.set_walking_mode)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        self.controller.subscribe_to_telemetrics(self.publish_telemetrics_data)

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
