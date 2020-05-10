from __future__ import division
import rospy
from hopper_controller.msg import HexapodLegPositions, MotorTorque
from hopper_controller.srv import ReadHexapodLegPositions
from hexapod import LegPositions, Vector3
import socket
import json


class IkController(object):
    def __init__(self):
        super(IkController, self).__init__()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def disable_motors(self):
        message = {
            "command": "DisableMotors",
        }
        payload = json.dumps(message)
        self.udp_socket.sendto(payload, ("127.0.0.1", 6666))

    def move_legs_synced(self, leg_positions):
        message = {
            "command": {
                "MoveTo": {
                "left_front": [
                    (leg_positions.left_front.x / 1000),
                    (leg_positions.left_front.y / 1000),
                    (leg_positions.left_front.z / 1000),
                ],
                "left_middle": [
                    (leg_positions.left_middle.x / 1000),
                    (leg_positions.left_middle.y / 1000),
                    (leg_positions.left_middle.z / 1000),
                ],
                "left_rear": [
                    (leg_positions.left_rear.x / 1000),
                    (leg_positions.left_rear.y / 1000),
                    (leg_positions.left_rear.z / 1000),
                ],
                "right_front": [
                    (leg_positions.right_front.x / 1000),
                    (leg_positions.right_front.y / 1000),
                    (leg_positions.right_front.z / 1000),
                ],
                "right_middle": [
                    (leg_positions.right_middle.x / 1000),
                    (leg_positions.right_middle.y / 1000),
                    (leg_positions.right_middle.z / 1000),
                ],
                "right_rear": [
                    (leg_positions.right_rear.x / 1000),
                    (leg_positions.right_rear.y / 1000),
                    (leg_positions.right_rear.z / 1000),
                ]
                }
                }
            }
        payload = json.dumps(message)
        self.udp_socket.sendto(payload, ("127.0.0.1", 6666))

    def read_current_leg_positions(self):
        message = {
            "command": "ReadPosition",
        }
        payload = json.dumps(message)
        self.udp_socket.sendto(payload, ("127.0.0.1", 6666))
        json_data, addr = self.udp_socket.recvfrom(1024)
        data = json.loads(json_data)
        def make_leg(data):
            return Vector3(data[0], data[1], data[2])
        return LegPositions(
            make_leg(data["left_front"]),
            make_leg(data["right_front"]),
            make_leg(data["left_middle"]),
            make_leg(data["right_middle"]),
            make_leg(data["left_rear"]),
            make_leg(data["right_rear"]),
        )
