#!/usr/bin/env python

from threading import Lock
import rospy

from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics
from hopper_controller.msg import BodyMotorPositions, MotorCompSpeedCommand, TorqueCommand
from hopper_controller.srv import ReadMotorPosition, ReadMotorPositionResponse
from dynamixel.dynamixel_driver import DynamixelDriver, search_usb_2_ax_port


class BodyMotorController(object):
    def __init__(self):
        super(BodyMotorController, self).__init__()
        rospy.init_node('hopper_body_controller', disable_signals=True)
        self.driver_lock = Lock()
        self.servo_driver = DynamixelDriver(search_usb_2_ax_port())
        self.servo_ids = self.servo_driver.search_servos(0, 20)
        rospy.Subscriber("hopper/body/motor_command", BodyMotorPositions, self.on_motor_command)
        rospy.Subscriber("hopper/body/compliance_speed", MotorCompSpeedCommand, self.on_compliance_speed_command)
        rospy.Subscriber("hopper/body/torque_command", TorqueCommand, self.on_torque_command)
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
        commands = map(lambda goal_command: (goal_command.servo_id, goal_command.goal_postion), msg.goal_postions)
        self.driver_lock.acquire()
        self.servo_driver.group_sync_write_goal_degrees(commands)
        self.driver_lock.release()

    def on_compliance_speed_command(self, command):
        self.driver_lock.acquire()
        self.servo_driver.set_compliance_slope(command.servo_id, command.compliance_slope)
        self.servo_driver.set_moving_speed(command.servo_id, command.movement_speed)
        self.driver_lock.release()

    def on_torque_command(self, command):
        self.driver_lock.acquire()
        self.servo_driver.set_torque(command.servo_id, command.toruqe_on)
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
