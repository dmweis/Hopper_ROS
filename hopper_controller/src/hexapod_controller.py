#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics
from std_msgs.msg import String

from hexapod.hexapod_gait_engine import GaitEngine, MovementController, TripodGait
from hexapod.hexapod_ik_driver import IkDriver, Vector2, Vector3
from dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        servo_driver = DynamixelDriver(search_usb_2_ax_port())
        ik_driver = IkDriver(servo_driver)
        tripod_gait = TripodGait(ik_driver)
        gait_engine = GaitEngine(tripod_gait)
        self.controller = MovementController(gait_engine)
        self.telemetrics_publisher = rospy.Publisher('hopper_telemetrics', HexapodTelemetrics, queue_size=5)
        rospy.Subscriber("hopper_move_command", Twist, self.update_direction)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        self.controller.subscribe_to_telemetrics(self.publish_telemetrics_data)

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
