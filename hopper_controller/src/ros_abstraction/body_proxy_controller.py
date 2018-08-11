import rospy
from hopper_controller.msg import *
from hopper_controller.srv import *


class HexapodBodyController(object):
    def __init__(self):
        super(HexapodBodyController, self).__init__()
        self.fk_disabled = rospy.get_param("~fk_demo_mode", False)
        if not self.fk_disabled:
            rospy.wait_for_service("hopper/read_hexapod_motor_positions")
        self.motor_commands = rospy.Publisher("hopper/body/motor_command", HexapodMotorPositions, queue_size=10)
        self.compliance_publisher = rospy.Publisher("hopper/body/motor_compliance", MotorCompliance, queue_size=5, latch=True)
        self.speed_publisher = rospy.Publisher("hopper/body/motor_speed", MotorSpeed, queue_size=5, latch=True)
        self.torque_publisher = rospy.Publisher("hopper/body/motor_torque", MotorTorque, queue_size=5, latch=True)
        if not self.fk_disabled:
            self.read_hexapod_motor_positions_proxy = rospy.ServiceProxy("hopper/read_hexapod_motor_positions", ReadHexapodMotorPositions, persistent=True)

    def set_motor_compliance(self, compliance):
        self.compliance_publisher.publish(MotorCompliance(compliance))

    def set_motor_speed(self, speed):
        self.speed_publisher.publish(MotorSpeed(speed))

    def set_torque(self, torque):
        self.torque_publisher.publish(MotorTorque(torque))

    def set_motors(self, positions):
        self.motor_commands.publish(positions)

    def read_hexapod_motor_positions(self):
        if self.fk_disabled:
            leg_motors = LegMotorPositions(150, 150, 150)
            return HexapodMotorPositions(leg_motors, leg_motors, leg_motors, leg_motors, leg_motors, leg_motors)
        return self.read_hexapod_motor_positions_proxy().motor_positions
