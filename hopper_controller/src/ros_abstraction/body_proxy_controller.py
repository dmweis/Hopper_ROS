import rospy
from hopper_controller.msg import *
from hopper_controller.srv import *


class HexapodBodyController(object):
    def __init__(self):
        super(HexapodBodyController, self).__init__()
        self.fk_disabled = rospy.get_param("~fk_demo_mode", False)
        if not self.fk_disabled:
            rospy.wait_for_service("hopper/read_motor_position")
        self.motor_commands = rospy.Publisher("hopper/body/motor_command", HopperMotorPositions, queue_size=10)
        self.compliance_publisher = rospy.Publisher("hopper/body/motor_compliance", MotorCompliance, queue_size=5, latch=True)
        self.speed_publisher = rospy.Publisher("hopper/body/motor_speed", MotorSpeed, queue_size=5, latch=True)
        self.torque_publisher = rospy.Publisher("hopper/body/motor_torque", MotorTorque, queue_size=5, latch=True)
        if not self.fk_disabled:
            self.read_motor_position_proxy = rospy.ServiceProxy("hopper/read_motor_position", ReadMotorPosition, persistent=True)

    def set_motor_compliance(self, compliance):
        self.compliance_publisher.publish(MotorCompliance(compliance))

    def set_motor_speed(self, speed):
        self.speed_publisher.publish(MotorSpeed(speed))

    def set_torque(self, torque):
        self.torque_publisher.publish(MotorTorque(torque))

    def set_motors(self, positions):
        self.motor_commands.publish(positions)

    def read_motor_position(self, servo_id):
        if self.fk_disabled:
            return 150
        return self.read_motor_position_proxy(servo_id).servo_position
