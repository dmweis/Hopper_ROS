import rospy
from hopper_controller.msg import *
from hopper_controller.srv import *


class HexapodBodyController(object):
    def __init__(self):
        rospy.wait_for_service("hopper/read_motor_position")
        self.motor_commands = rospy.Publisher("hopper/body/motor_command", BodyMotorPositions, queue_size=5)
        self.compliance_speed = rospy.Publisher("hopper/body/compliance_speed", MotorCompSpeedCommand, queue_size=5)
        self.torque_command = rospy.Publisher("hopper/body/torque_command", TorqueCommand, queue_size=5)
        self.read_motor_position_proxy = rospy.ServiceProxy("hopper/read_motor_position", ReadMotorPosition, persistent=True)

    def set_compliance_movement_speed(self, servo_id, compliance, speed):
        self.compliance_speed.publish(MotorCompSpeedCommand(servo_id, compliance, speed))

    def set_torque(self, servo_id, value):
        self.torque_command.publish(TorqueCommand(servo_id, value))

    def set_motors(self, positions):
        motor_positions = BodyMotorPositions()
        for servo_id, position in positions:
            motor_positions.goal_postions.append(MotorGoalPosition(servo_id, position))
        self.motor_commands.publish(motor_positions)

    def read_motor_position(self, servo_id):
        return self.read_motor_position_proxy(servo_id).servo_position
