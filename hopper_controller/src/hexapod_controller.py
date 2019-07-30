#!/usr/bin/env python

from __future__ import division
import math
import rospy
from geometry_msgs.msg import Twist
from hopper_msgs.msg import HopperMoveCommand, HaltCommand
from hopper_controller.msg import SingleLegCommand, StandCommand, FoldCommand
from std_msgs.msg import String, Empty
from std_srvs.srv import Empty as EmptySrv


from hexapod.hexapod_gait_engine import GaitEngine, MovementController, TripodGait
from hexapod.hexapod_ik_driver import Vector2, Vector3
from hexapod.folding_manager import FoldingManager
import ros_abstraction


def linear_map(value, inMin, inMax, outMin, outMax):
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def constrain(value, a, b):
    min_value = min(a, b)
    max_value = max(a, b)
    return min(max_value, max(min_value, value))


class HexapodController(object):
    def __init__(self):
        super(HexapodController, self).__init__()
        rospy.init_node('hopper_controller')
        # build controller
        self.sound_on = rospy.get_param("~sound_on", True)
        ik_driver = ros_abstraction.IkController()
        body_controller = ros_abstraction.HexapodBodyController()
        message_publisher = ros_abstraction.MessagePublisher()
        controller_telemetry = ros_abstraction.ControllerTelemetryPublisher()
        lidar_controller = ros_abstraction.LidarController()
        orientation_publisher = ros_abstraction.BodyOrientationPublisher()
        tripod_gait = TripodGait(ik_driver, ros_abstraction.HeightPublisher(message_publisher),
                                 ros_abstraction.OdomPublisher(message_publisher, ros_abstraction.ImuReader()))
        gait_engine = GaitEngine(tripod_gait)
        leg_controller = ros_abstraction.LegController(gait_engine)
        folding_manager = FoldingManager(body_controller)
        self.controller = MovementController(gait_engine, ros_abstraction.SoundPlayer(self.sound_on), folding_manager, controller_telemetry, leg_controller, lidar_controller)
        self.halt_publisher = rospy.Publisher("halt", Empty, queue_size=1, latch=True)
        rospy.Subscriber("hopper/cmd_vel", Twist, self.on_nav_system_move_command)
        rospy.Subscriber("hopper/move_command", HopperMoveCommand, self.on_move_command)
        rospy.Subscriber("hopper/stance_translate", Twist, self.update_pose)
        rospy.Subscriber("hopper/single_leg_command", SingleLegCommand, self.update_single_leg)
        rospy.Subscriber("hopper_schedule_move", String, self.schedule_move)
        rospy.Subscriber("hopper/halt", HaltCommand, self.on_halt_command)
        rospy.Subscriber("hopper/stand", StandCommand, self.on_stand_command, queue_size=10)
        rospy.Subscriber("hopper/fold_command", FoldCommand, self.on_fold_command, queue_size=10)
        rospy.Service("hopper/step_to_relaxed", EmptySrv, self.on_step_to_relaxed)
        self.controller.spin()
        message_publisher.stop()
        orientation_publisher.stop()
        self.halt_publisher.publish(Empty())

    def on_halt_command(self, _):
        self.controller.keep_running = False

    def on_stand_command(self, stand_command):
        self.controller.stand_mode = stand_command.stand

    def on_fold_command(self, msg):
        self.controller.folding_operation = msg.operation

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
        # max_theta_vel = rospy.get_param("hopper/max_theta_vel", default=2)
        # max_vel = rospy.get_param("hopper/max_linear_vel", default=0.3)
        max_theta_vel = 2
        max_vel = 0.3
        speed = math.sqrt(move_command.linear.x**2 + move_command.linear.y **2)
        cycle_time = 1.0
        if abs(speed) > 0.05:
            cycle_time = linear_map(speed, 0, 0.3, 1, 0.25)
        elif abs(move_command.angular.z) > 0.1:
            cycle_time = linear_map(speed, 0, 2, 1, 0.25)
        cycle_time = constrain(cycle_time, 1, 0.25)

        if abs(move_command.angular.z) > max_theta_vel:
            rospy.logerr("Max angular vel {0:.2f} was breached to {1:.2f}".format(max_theta_vel, move_command.angular.z))
            move_command.angular.z = math.copysign(max_theta_vel, move_command.angular.z)
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
                                         cycle_time,
                                         HopperMoveCommand.DEFAULT_LIFT_HEIGHT)

    def update_pose(self, twist):
        transform = Vector3(twist.linear.x * 100, twist.linear.y * 100, twist.linear.z * 100)
        rotation = Vector3(math.degrees(twist.angular.x), math.degrees(twist.angular.y), math.degrees(twist.angular.z))
        self.controller.set_relaxed_pose(transform, rotation)

    def update_single_leg(self, msg):
        position = Vector3(msg.position.x * 100, msg.position.y * 100, msg.position.z * 100)
        self.controller.update_single_leg_command(msg.selected_leg, position, msg.single_leg_mode_on, msg.fast_mode)

    def schedule_move(self, move_name):
        self.controller.schedule_move(move_name.data)

    def on_step_to_relaxed(self, srv):
        self.controller.execute_step_to_relaxed()


if __name__ == '__main__':
    HexapodController()
