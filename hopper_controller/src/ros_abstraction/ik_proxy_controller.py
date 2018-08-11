import rospy
from hopper_controller.msg import HexapodLegPositions, MotorTorque
from hopper_controller.srv import ReadHexapodLegPositions
from hexapod import LegPositions


class IkController(object):
    def __init__(self):
        super(IkController, self).__init__()
        rospy.wait_for_service("hopper/ik/read_leg_positions")
        self.read_legs_service_proxy = rospy.ServiceProxy("hopper/ik/read_leg_positions", ReadHexapodLegPositions)
        self. move_legs_pub = rospy.Publisher("hopper/ik/move_legs", HexapodLegPositions, queue_size=5, latch=True)
        self.torque_publisher = rospy.Publisher("hopper/body/motor_torque", MotorTorque, queue_size=5, latch=True)

    def disable_motors(self):
        self.torque_publisher.publish(MotorTorque(False))

    def move_legs_synced(self, leg_positions):
        self.move_legs_pub.publish(leg_positions)

    def read_current_leg_positions(self):
        leg_positions_msg = self.read_current_leg_positions()
        return LegPositions.ros_leg_positions_to_leg_positions(leg_positions_msg)
