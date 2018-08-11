#!/usr/bin/env python

from threading import Lock
import rospy

from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics
from hopper_controller.msg import HopperMotorPositions, MotorCompliance, MotorSpeed, MotorTorque
from hopper_controller.srv import ReadMotorPosition, ReadMotorPositionResponse
from dynamixel import DynamixelDriver, search_usb_2_ax_port


class BodyMotorController(object):
    def __init__(self):
        super(BodyMotorController, self).__init__()
        rospy.init_node('hopper_body_controller')
        self.driver_lock = Lock()
        self.leg_data = rospy.get_param("legs")
        self.servo_ids = []
        for leg in self.leg_data:
            self.servo_ids.append(leg["coxa_id"])
            self.servo_ids.append(leg["femur_id"])
            self.servo_ids.append(leg["tibia_id"])
        self.servo_driver = DynamixelDriver(search_usb_2_ax_port())
        rospy.Subscriber("hopper/body/motor_command", HopperMotorPositions, self.on_motor_command, queue_size=20)
        rospy.Subscriber("hopper/body/motor_compliance", MotorCompliance, self.on_compliance_command, queue_size=25)
        rospy.Subscriber("hopper/body/motor_speed", MotorSpeed, self.on_speed_command, queue_size=5)
        rospy.Subscriber("hopper/body/motor_torque", MotorTorque, self.on_torque_command, queue_size=20)
        self.motor_positions_reader_service_id = rospy.Service("hopper/read_motor_position", ReadMotorPosition, self.read_motor_position)
        self.telementrics_publisher = rospy.Publisher('hopper_telemetrics', HexapodTelemetrics, queue_size=5)
        duration = rospy.Duration(4)
        while not rospy.is_shutdown():
            rospy.sleep(duration)
            self.read_motor_telemetrics()
        self.driver_lock.acquire()
        for servo_id in self.servo_ids:
            self.servo_driver.set_torque(servo_id, False)
        self.servo_driver.close()
        self.driver_lock.release()

    def on_motor_command(self, msg):
        commands = [
            # left front
            (self.leg_data["left_front"]["coxa_id"], msg.left_front_coxa),
            (self.leg_data["left_front"]["femur_id"], msg.left_front_femur),
            (self.leg_data["left_front"]["coxa_id"], msg.left_front_tibia),
            # right front
            (self.leg_data["right_front"]["coxa_id"], msg.right_front_coxa),
            (self.leg_data["right_front"]["femur_id"], msg.right_front_femur),
            (self.leg_data["right_front"]["coxa_id"], msg.right_front_tibia),
            # left middle
            (self.leg_data["left_middle"]["coxa_id"], msg.left_middle_coxa),
            (self.leg_data["left_middle"]["femur_id"], msg.left_middle_femur),
            (self.leg_data["left_middle"]["coxa_id"], msg.left_middle_tibia),
            # right middle
            (self.leg_data["right_front"]["coxa_id"], msg.right_front_coxa),
            (self.leg_data["right_front"]["femur_id"], msg.right_front_femur),
            (self.leg_data["right_front"]["coxa_id"], msg.right_front_tibia),
            # left rear
            (self.leg_data["left_rear"]["coxa_id"], msg.left_rear_coxa),
            (self.leg_data["left_rear"]["femur_id"], msg.left_rear_femur),
            (self.leg_data["left_rear"]["coxa_id"], msg.left_rear_tibia),
            # right rear
            (self.leg_data["right_rear"]["coxa_id"], msg.right_rear_coxa),
            (self.leg_data["right_rear"]["femur_id"], msg.right_rear_femur),
            (self.leg_data["right_rear"]["coxa_id"], msg.right_rear_tibia)
        ]
        self.driver_lock.acquire()
        self.servo_driver.group_sync_write_goal_degrees(commands)
        self.driver_lock.release()

    def on_compliance_command(self, command):
        self.driver_lock.acquire()
        for servo_id in self.servo_ids:
            self.servo_driver.set_compliance_slope(servo_id, command.compliance)
        self.driver_lock.release()

    def on_speed_command(self, command):
        self.driver_lock.acquire()
        for servo_id in self.servo_ids:
            self.servo_driver.set_moving_speed(servo_id, command.speed)
        self.driver_lock.release()

    def on_torque_command(self, command):
        self.driver_lock.acquire()
        for servo_id in self.servo_ids:
            self.servo_driver.set_torque(servo_id, command.torque)
        self.driver_lock.release()

    def read_motor_position(self, read_command):
        self.driver_lock.acquire()
        pos = self.servo_driver.read_current_position_degrees(read_command.servo_id)
        self.driver_lock.release()
        return ReadMotorPositionResponse(pos)

    def read_motor_telemetrics(self):
        robot_telemetrics = HexapodTelemetrics()
        for servo_id in self.servo_ids:
            self.driver_lock.acquire()
            voltage = self.servo_driver.read_voltage(servo_id)
            temperature = self.servo_driver.read_temperature(servo_id)
            self.driver_lock.release()
            robot_telemetrics.servos.append(ServoTelemetrics(servo_id, temperature, voltage))
        self.telementrics_publisher.publish(robot_telemetrics)


if __name__ == '__main__':
    BodyMotorController()
