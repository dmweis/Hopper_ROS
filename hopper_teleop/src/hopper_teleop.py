#!/usr/bin/env python

import math
import rospy
from hopper_msgs.msg import HopperMoveCommand
import pyglet

class HopperTeleop(pyglet.window.Window):
    def __init__(self):
        super(HopperTeleop, self).__init__()
        rospy.init_node('hopper_teleop', disable_signals=True)
        self.publisher = rospy.Publisher('hopper/move_command', HopperMoveCommand, queue_size=10)
        self.x = 0
        self.y = 0
        self.rotation = 0
        try:
            pyglet.app.run()
        except KeyboardInterrupt:
            pass

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.W:
            self.x += 1
        if symbol == pyglet.window.key.S:
            self.x -= 1
        if symbol == pyglet.window.key.A:
            self.y += 1
        if symbol == pyglet.window.key.D:
            self.y -= 1
        if symbol == pyglet.window.key.Q:
            self.rotation += 1
        if symbol == pyglet.window.key.E:
            self.rotation -= 1
        if symbol == pyglet.window.key.ESCAPE:
            pyglet.app.exit()
        self.send_new_message()

    def on_key_release(self, symbol, modifiers):
        if symbol == pyglet.window.key.W:
            self.x -= 1
        if symbol == pyglet.window.key.S:
            self.x += 1
        if symbol == pyglet.window.key.A:
            self.y -= 1
        if symbol == pyglet.window.key.D:
            self.y += 1
        if symbol == pyglet.window.key.Q:
            self.rotation -= 1
        if symbol == pyglet.window.key.E:
            self.rotation += 1
        self.send_new_message()

    def send_new_message(self):
        new_message = HopperMoveCommand()
        new_message.lift_height = 4
        new_message.cycle_time = 0.75
        new_message.direction.linear.x = self.x * 0.12
        new_message.direction.linear.y = self.y * 0.12
        new_message.direction.angular.z = math.radians(self.rotation * 30)
        self.publisher.publish(new_message)

if __name__ == '__main__':
    HopperTeleop()
