#!/usr/bin/env python

from __future__ import division
import math
import rospy
from geometry_msgs.msg import Twist
from hopper_msgs.msg import HopperMoveCommand, HaltCommand
from hopper_keep_alive.srv import Halt
from std_msgs.msg import String


from hexapod.hexapod_gait_engine import GaitEngine, MovementController, TripodGait
from hexapod.hexapod_ik_driver import IkDriver, Vector2, Vector3
import ros_abstraction


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        body_controller = ros_abstraction.HexapodBodyController()
        # build controller
        ik_driver = IkDriver(body_controller, ros_abstraction.JointStatePublisher())
        tripod_gait = TripodGait(ik_driver, ros_abstraction.HeightPublisher(), ros_abstraction.OdomPublisher())
        gait_engine = GaitEngine(tripod_gait)
        self.controller = MovementController(gait_engine, ros_abstraction.SoundPlayer())
        self.halt_service = rospy.ServiceProxy("halt", Halt)
        rospy.Subscriber("hopper/cmd_vel", Twist, self.on_nav_system_move_command)
        rospy.Subscriber("hopper/move_command", HopperMoveCommand, self.on_move_command)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        rospy.Subscriber("hopper/halt", HaltCommand, self.on_halt_command)
        self.controller.spin()

    def on_halt_command(self, _):
        self.controller.keep_running = False
        self.halt_service()

    def on_move_command(self, move_command):
        # convert directions from meter to cm
        direction = Vector2(move_command.direction.linear.x, move_command.direction.linear.y) * 100
        rotation = move_command.direction.angular.z
        self.controller.set_move_command(direction,
                                         math.degrees(rotation),
                                         move_command.cycle_time,
                                         move_command.lift_height)

    def on_nav_system_move_command(self, move_command):
        # convert directions from meter to cm
        max_theta = 0.2
        max_vel = 0.1
        if abs(move_command.angular.z) > max_theta:
            rospy.logerr("Max angular vel {0:.2f} was breached to {1:.2f}".format(max_theta, move_command.angular.z))
            move_command.angular.z = math.copysign(max_theta, move_command.angular.z)
        if abs(move_command.linear.x) > max_vel:
            rospy.logerr("Max linear x vel {0:.2f} was breached to {1:.2f}".format(max_vel, move_command.linear.x))
            move_command.linear.x = math.copysign(max_vel, move_command.linear.x)
        if abs(move_command.linear.y) > max_vel:
            rospy.logerr("Max linear y vel {0:.2f} was breached to {1:.2f}".format(max_vel, move_command.linear.y))
            move_command.linear.y = math.copysign(max_vel, move_command.linear.y)
        velocity = Vector2(move_command.linear.x, move_command.linear.y) * 100
        rotation = move_command.angular.z
        self.controller.set_move_command(velocity,
                                         math.degrees(rotation),
                                         HopperMoveCommand.DEFAULT_CYCLE_TIME,
                                         HopperMoveCommand.DEFAULT_LIFT_HEIGHT)

    def update_pose(self, twist):
        transform = Vector3(twist.linear.x, twist.linear.y, twist.linear.z)
        rotation = Vector3(twist.angular.x, twist.angular.y, twist.angular.z)
        self.controller.set_relaxed_pose(transform, rotation)

    def schedule_move(self, move_name):
        self.controller.schedule_move(move_name.data)


if __name__ == '__main__':
    HexapodController()
