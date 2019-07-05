#!/usr/bin/env python

from threading import Lock
import rospy

from hopper_msgs.msg import ServoTelemetry, HexapodTelemetry
from hopper_controller.msg import HexapodMotorPositions, LegMotorPositions, MotorCompliance, MotorSpeed, MotorTorque
from hopper_controller.srv import ReadHexapodMotorPositions, ReadHexapodMotorPositionsResponse
from dynamixel import DynamixelDriver, search_usb_2_ax_port
from ros_abstraction import JointStatePublisher, MessagePublisher


class BodyMotorController(object):
    def __init__(self):
        super(BodyMotorController, self).__init__()
        rospy.init_node('hopper_body_controller')
        self.driver_lock = Lock()
        self.leg_data = rospy.get_param("legs")
        self.servo_ids = []
        for leg in self.leg_data:
            self.servo_ids.append(self.leg_data[leg]["coxa_id"])
            self.servo_ids.append(self.leg_data[leg]["femur_id"])
            self.servo_ids.append(self.leg_data[leg]["tibia_id"])
        self.servo_driver = DynamixelDriver(search_usb_2_ax_port())
        self.message_publisher = MessagePublisher()
        self.joint_state_publisher = JointStatePublisher(self.message_publisher)
        rospy.Subscriber("hopper/body/motor_command", HexapodMotorPositions, self.on_motor_command, queue_size=20)
        rospy.Subscriber("hopper/body/motor_compliance", MotorCompliance, self.on_compliance_command, queue_size=25)
        rospy.Subscriber("hopper/body/motor_speed", MotorSpeed, self.on_speed_command, queue_size=5)
        rospy.Subscriber("hopper/body/motor_torque", MotorTorque, self.on_torque_command, queue_size=20)
        self.body_motor_positions_service = rospy.Service("hopper/read_hexapod_motor_positions", ReadHexapodMotorPositions, self.read_hexapod_motor_positions)
        self.telementrics_publisher = rospy.Publisher('hopper_telemetry', HexapodTelemetry, queue_size=5)
        telemetry_update_interval = rospy.Duration.from_sec(0.1)
        self.telemetry_motor_id_index = 0
        while not rospy.is_shutdown():
            rospy.sleep(telemetry_update_interval)
            try:
                self.read_motor_telemetry()
            except IOError as e:
                rospy.logerr("IOError on telemetry read " + str(e))
        with self.driver_lock:
            for servo_id in self.servo_ids:
                self.servo_driver.set_torque(servo_id, False)
            self.servo_driver.close()

    def on_motor_command(self, msg):
        commands = [
            # left front
            (self.leg_data["left_front"]["coxa_id"], msg.left_front.coxa),
            (self.leg_data["left_front"]["femur_id"], msg.left_front.femur),
            (self.leg_data["left_front"]["tibia_id"], msg.left_front.tibia),
            # right front
            (self.leg_data["right_front"]["coxa_id"], msg.right_front.coxa),
            (self.leg_data["right_front"]["femur_id"], msg.right_front.femur),
            (self.leg_data["right_front"]["tibia_id"], msg.right_front.tibia),
            # left middle
            (self.leg_data["left_middle"]["coxa_id"], msg.left_middle.coxa),
            (self.leg_data["left_middle"]["femur_id"], msg.left_middle.femur),
            (self.leg_data["left_middle"]["tibia_id"], msg.left_middle.tibia),
            # right middle
            (self.leg_data["right_middle"]["coxa_id"], msg.right_middle.coxa),
            (self.leg_data["right_middle"]["femur_id"], msg.right_middle.femur),
            (self.leg_data["right_middle"]["tibia_id"], msg.right_middle.tibia),
            # left rear
            (self.leg_data["left_rear"]["coxa_id"], msg.left_rear.coxa),
            (self.leg_data["left_rear"]["femur_id"], msg.left_rear.femur),
            (self.leg_data["left_rear"]["tibia_id"], msg.left_rear.tibia),
            # right rear
            (self.leg_data["right_rear"]["coxa_id"], msg.right_rear.coxa),
            (self.leg_data["right_rear"]["femur_id"], msg.right_rear.femur),
            (self.leg_data["right_rear"]["tibia_id"], msg.right_rear.tibia)
        ]
        with self.driver_lock:
            self.servo_driver.group_sync_write_goal_degrees(commands)
        self.joint_state_publisher.update_joint_states(msg)

    def on_compliance_command(self, command):
        with self.driver_lock:
            for servo_id in self.servo_ids:
                self.servo_driver.set_compliance_slope(servo_id, command.compliance)

    def on_speed_command(self, command):
        with self.driver_lock:
            for servo_id in self.servo_ids:
                self.servo_driver.set_moving_speed(servo_id, command.speed)

    def on_torque_command(self, command):
        with self.driver_lock:
            for servo_id in self.servo_ids:
                self.servo_driver.set_torque(servo_id, command.torque)

    def read_hexapod_motor_positions(self, _):
        def read_pos(servo_id):
            return self.servo_driver.read_current_position_degrees(servo_id)

        def read_leg(leg_config):
            return LegMotorPositions(
                read_pos(leg_config["coxa_id"]),
                read_pos(leg_config["femur_id"]),
                read_pos(leg_config["tibia_id"])
            )
        msg = HexapodMotorPositions()
        with self.driver_lock:
            msg.left_front = read_leg(self.leg_data["left_front"])
            msg.right_front = read_leg(self.leg_data["right_front"])
            msg.left_middle = read_leg(self.leg_data["left_middle"])
            msg.right_middle = read_leg(self.leg_data["right_middle"])
            msg.left_rear = read_leg(self.leg_data["left_rear"])
            msg.right_rear = read_leg(self.leg_data["right_rear"])
        return ReadHexapodMotorPositionsResponse(msg)

    def read_motor_telemetry(self):
        robot_telemetry = HexapodTelemetry()
        with self.driver_lock:
            servo_id = self.servo_ids[self.telemetry_motor_id_index]
            voltage = self.servo_driver.read_voltage(servo_id)
            temperature = self.servo_driver.read_temperature(servo_id)
            robot_telemetry.servos.append(ServoTelemetry(servo_id, temperature, voltage))
        self.telemetry_motor_id_index += 1
        if self.telemetry_motor_id_index > len(self.servo_ids) - 1:
            self.telemetry_motor_id_index = 0
        self.telementrics_publisher.publish(robot_telemetry)


if __name__ == '__main__':
    BodyMotorController()
