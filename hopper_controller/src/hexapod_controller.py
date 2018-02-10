#!/usr/bin/env python

from __future__ import absolute_import
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from hexapod_gait_engine import GaitController
from hexapod_ik_driver import IkDriver, Vector2, Vector3
from dynamixel_driver import DynamixelDriver, search_usb_2_ax_port
from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics

class HexapodController(object):
    def __init__(self):
        servo_driver = DynamixelDriver(search_usb_2_ax_port())
        ik_driver = IkDriver(servo_driver)
        self.controller = GaitController(ik_driver)
        self.telemetrics_publisher = rospy.Publisher('hopper_telemetrics', HexapodTelemetrics, queue_size=10)
        self.controller.telemetrics_callback = self.publish_telemetrics_data

    def update_direction(self, twist):
        self.controller.direction = Vector2(twist.linear.x, twist.linear.y)
        self.controller.rotation = twist.angular.x

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

    def update_stance(self, twist):
        transform = Vector3(twist.linear.x, twist.linear.y, twist.linear.z)
        rotation = Vector3(twist.angular.x, twist.angular.y, twist.angular.z)
        self.controller.update_relaxed_position(transform=transform, rotation=rotation)

    def listener(self):
        rospy.init_node('hopper_controller', anonymous=True)
        rospy.Subscriber("quadruped_command", Twist, self.update_direction)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_stance)
        rospy.spin()
        self.controller.stop()


if __name__ == '__main__':
    HexapodController().listener()
