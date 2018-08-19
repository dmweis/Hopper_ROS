#!/usr/bin/env python

import rospy

from hexapod import IkDriver, LegPositions
from ros_abstraction import HexapodBodyController, JointStatePublisher, MessagePublisher
from hopper_controller.msg import HexapodLegPositions
from hopper_controller.srv import ReadHexapodLegPositions, ReadHexapodLegPositionsResponse


class IkControllerNode(object):
    def __init__(self):
        super(IkControllerNode, self).__init__()
        rospy.init_node("hopper_ik_node")
        body_controller_proxy = HexapodBodyController()
        self.message_publisher = MessagePublisher()
        joint_state_publisher = JointStatePublisher(self.message_publisher)
        self.ik_driver = IkDriver(body_controller_proxy, joint_state_publisher)
        rospy.Subscriber("hopper/ik/move_legs", HexapodLegPositions, self.on_move_legs, queue_size=5)
        self.read_leg_positions_service = rospy.Service("hopper/ik/read_leg_positions", ReadHexapodLegPositions, self.read_hexapod_leg_positions)
        self.ik_driver.setup()
        self.ik_driver.disable_motors()
        rospy.spin()
        self.message_publisher.stop()
        self.ik_driver.disable_motors()

    def read_hexapod_leg_positions(self, _):
        leg_positions_msg = HexapodLegPositions()
        leg_positions = self.ik_driver.read_current_leg_positions()
        leg_positions_msg.left_front = leg_positions.left_front
        leg_positions_msg.right_front = leg_positions.right_front
        leg_positions_msg.left_middle = leg_positions.left_middle
        leg_positions_msg.right_middle = leg_positions.right_middle
        leg_positions_msg.left_rear = leg_positions.left_rear
        leg_positions_msg.right_rear = leg_positions.right_rear
        return ReadHexapodLegPositionsResponse(leg_positions_msg)

    def on_move_legs(self, leg_positions):
        hopper_leg_positions = LegPositions.ros_leg_positions_to_leg_positions(leg_positions)
        self.ik_driver.move_legs_synced(hopper_leg_positions)


if __name__ == "__main__":
    IkControllerNode()