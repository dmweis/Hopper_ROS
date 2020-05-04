import rospy
from hopper_controller.msg import *
from hopper_controller.srv import *
import socket
import json
from math import degrees, radians


class HexapodBodyController(object):
    def __init__(self):
        super(HexapodBodyController, self).__init__()
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_motor_compliance(self, compliance):
        message = {
            "command": {
                "SetCompliance": compliance
            }
        }
        payload = json.dumps(message)
        self.sock.sendto(payload, ("127.0.0.1", 6666))

    def set_motor_speed(self, speed):
        message = {
            "command": {
                "SetSpeed": compliance
            }
        }
        payload = json.dumps(message)
        self.sock.sendto(payload, ("127.0.0.1", 6666))

    def set_torque(self, torque):
        message = {
            "command": {
                "SetTorque": torque
            }
        }
        payload = json.dumps(message)
        self.sock.sendto(payload, ("127.0.0.1", 6666))

    def set_motors(self, positions):
        message = {
            "command": {
                "MoveTo": {
                "left_front": {
                    "coxa": radians(positions.left_front.coxa),
                    "femur": radians(positions.left_front.femur),
                    "tibia": radians(positions.left_front.tibia),
                },
                "left_middle": {
                    "coxa": radians(positions.left_middle.coxa),
                    "femur": radians(positions.left_middle.femur),
                    "tibia": radians(positions.left_middle.tibia),
                },
                "left_rear": {
                    "coxa": radians(positions.left_rear.coxa),
                    "femur": radians(positions.left_rear.femur),
                    "tibia": radians(positions.left_rear.tibia),
                },
                "right_front": {
                    "coxa": radians(positions.right_front.coxa),
                    "femur": radians(positions.right_front.femur),
                    "tibia": radians(positions.right_front.tibia),
                },
                "right_middle": {
                    "coxa": radians(positions.right_middle.coxa),
                    "femur": radians(positions.right_middle.femur),
                    "tibia": radians(positions.right_middle.tibia),
                },
                "right_rear": {
                    "coxa": radians(positions.right_rear.coxa),
                    "femur": radians(positions.right_rear.femur),
                    "tibia": radians(positions.right_rear.tibia),
                }
                }
                }
            }
        payload = json.dumps(message)
        self.sock.sendto(payload, ("127.0.0.1", 6666))

    def read_hexapod_motor_positions(self):
        message = {
            "command": "ReadPosition",
        }
        payload = json.dumps(message)
        self.sock.sendto(payload, ("127.0.0.1", 6666))
        json, addr = sock.recvfrom(1024)
        data = json.loads(json)
        def make_leg(data):
            return LegMotorPositions(degrees(data["coxa"]), degrees(data["femur"]), degrees(data["tibia"]))
        return HexapodMotorPositions(
            make_leg(data["left_front"]),
            make_leg(data["right_front"]),
            make_leg(data["left_middle"]),
            make_leg(data["right_middle"]),
            make_leg(data["left_rear"]),
            make_leg(data["right_rear"]),
        )
