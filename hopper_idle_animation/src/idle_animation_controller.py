#!/usr/bin/env python

import random
import rospy

from hopper_msgs.msg import HopperMoveCommand
from std_msgs.msg import String

IDLE_ANIMATIONS = [
    "bored_looking_around"
]


class IdleAnimationController(object):
    def __init__(self):
        super(IdleAnimationController, self).__init__()
        rospy.init_node("hopper_idle_animation")
        self.last_action_time = rospy.Time.now()
        self.last_idle_action_time = rospy.Time.now()
        self.idle_timeout = rospy.Duration(10)
        self.idle_action_timeout = rospy.Duration(5)
        rospy.Subscriber("hopper/move_command", HopperMoveCommand,
                         self.on_move_command, queue_size=10)
        self.animation_publisher = rospy.Publisher(
            "hopper_schedule_move", String, queue_size=5)
        while not rospy.is_shutdown():
            self.idler_check()
            rospy.sleep(rospy.Duration(2))

    def on_move_command(self, msg):
        moving = msg.direction.linear.x != 0 or \
            msg.direction.linear.y != 0 or \
            msg.direction.linear.z != 0 or \
            msg.direction.angular.x != 0 or \
            msg.direction.angular.y != 0 or \
            msg.direction.angular.z != 0
        if moving:
            self.last_action_time = rospy.Time.now()

    def is_idle(self):
        now = rospy.Time.now()
        rospy.logdebug("Time since action:" + str((now - self.last_action_time).to_sec()))
        return now - self.last_action_time > self.idle_timeout and \
            now - self.last_idle_action_time > self.idle_action_timeout

    def idler_check(self):
        if self.is_idle():
            rospy.logdebug("Idle action executed")
            self.animation_publisher.publish(String(random.choice(IDLE_ANIMATIONS)))
            self.last_idle_action_time = rospy.Time.now()


if __name__ == "__main__":
    IdleAnimationController()
