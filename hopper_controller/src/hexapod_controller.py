#!/usr/bin/env python

from __future__ import division
import math
import rospy
from geometry_msgs.msg import Twist
from hopper_msgs.msg import HopperMoveCommand, HaltCommand
from hopper_controller.msg import SingleLegCommand
from std_msgs.msg import String, Empty


from hexapod.hexapod_gait_engine import GaitEngine, MovementController, TripodGait
from hexapod.hexapod_ik_driver import Vector2, Vector3
import ros_abstraction


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        # build controller
        self.sound_on = rospy.get_param("~sound_on", True)
        ik_driver = ros_abstraction.IkController()
        message_publisher = ros_abstraction.MessagePublisher()
        tripod_gait = TripodGait(ik_driver, ros_abstraction.HeightPublisher(message_publisher),
                                 ros_abstraction.OdomPublisher(message_publisher))
        gait_engine = GaitEngine(tripod_gait)
        self.controller = MovementController(gait_engine, ros_abstraction.SoundPlayer(self.sound_on))
        self.halt_publisher = rospy.Publisher("halt", Empty, queue_size=1, latch=True)
        rospy.Subscriber("hopper/cmd_vel", Twist, self.on_nav_system_move_command)
        rospy.Subscriber("hopper/move_command", HopperMoveCommand, self.on_move_command)
        rospy.Subscriber("hopper_stance_translate", Twist, self.update_pose_centimeters)
        rospy.Subscriber("hopper/stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper/single_leg_command", SingleLegCommand, self.update_single_leg)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        rospy.Subscriber("hopper/halt", HaltCommand, self.on_halt_command)
        self.controller.spin()
        message_publisher.stop()
        self.halt_publisher.publish(Empty())

    def on_halt_command(self, _):
        self.controller.keep_running = False

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
        transform = Vector3(twist.linear.x * 100, twist.linear.y * 100, twist.linear.z * 100)
        rotation = Vector3(math.degrees(twist.angular.x), math.degrees(twist.angular.y), math.degrees(twist.angular.z))
        self.controller.set_relaxed_pose(transform, rotation)

    def update_single_leg(self, msg):
        position = Vector3(msg.position.x * 100, msg.position.y * 100, msg.position.z * 100)
        self.controller.update_single_leg_command(msg.selected_leg, position, msg.single_leg_mode_on, msg.fast_mode)

    def update_pose_centimeters(self, twist):
        transform = Vector3(twist.linear.x, twist.linear.y, twist.linear.z)
        rotation = Vector3(twist.angular.x, twist.angular.y, twist.angular.z)
        self.controller.set_relaxed_pose(transform, rotation)

    def schedule_move(self, move_name):
        self.controller.schedule_move(move_name.data)


if __name__ == '__main__':
    HexapodController()
