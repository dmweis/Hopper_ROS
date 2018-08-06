#!/usr/bin/env python

from __future__ import division
import math
import struct
from threading import Thread
import rospy
from geometry_msgs.msg import Twist, Quaternion, Vector3
from hopper_msgs.msg import HopperMoveCommand, HaltCommand
from std_msgs.msg import String
from steamcontroller import SteamController, SCButtons, SCStatus, SCI_NULL
import usb1

def linear_map(value, inMin, inMax, outMin, outMax):
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

AXIS_MIN = -32768
AXIS_MAX = 32767

RIGHT_PAD = 0
LEFT_PAD = 1

def linear_map(value, inMin, inMax, outMin, outMax):
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def constrain(value, min_value, max_value):
    return min(max_value, max(min_value, value))

def remap_axis(x, y, left_pad=False, right_pad=False):
    x = linear_map(x, AXIS_MIN, AXIS_MAX, -1, 1)
    y = linear_map(y, AXIS_MIN, AXIS_MAX, -1, 1)
    if left_pad or right_pad:
        distance = math.hypot(x, y)
        corrected_angle = 0
        if left_pad:
            corrected_angle = math.atan2(x, y) + 0.1 * math.pi
        elif right_pad:
            corrected_angle = math.atan2(x, y) - 0.1 * math.pi
        x = math.sin(corrected_angle) * distance
        y = math.cos(corrected_angle) * distance
    x = constrain(x, -1, 1)
    y = constrain(y, -1, 1)
    return x, y

def scale_trigger(value):
    return linear_map(value, 0, 255, 0, 1)

def check_button(buttons, button_flag):
    return buttons & button_flag == button_flag


class RosSteamController(SteamController):
    def run(self):
        while not rospy.is_shutdown() and any(x.isSubmitted() for x in self._transfer_list):
            self._ctx.handleEvents()
            if len(self._cmsg) > 0:
                cmsg = self._cmsg.pop()
                self._sendControl(cmsg)


class SteamControllerRosHandler(object):
    def __init__(self):
        super(SteamControllerRosHandler, self).__init__()
        rospy.init_node("steam_controller_node")
        self._previous_controller_data = SCI_NULL

        self._left_pad_moved = 0
        self._right_pad_moved = 0

        self.pub = rospy.Publisher("hopper/move_command", HopperMoveCommand, queue_size=10)
        self.speech_pub = rospy.Publisher('hopper_play_sound', String, queue_size=5)
        self.move_pub = rospy.Publisher('hopper_schedule_move', String, queue_size=5)
        self.halt_command = rospy.Publisher('halt', HaltCommand, queue_size=1)
        self._hopper_move_command_msg = HopperMoveCommand()
        self._new_command_available = True
        self.sc = RosSteamController(self.on_controller_data)
        # activate IMU
        self.sc._sendControl(struct.pack('>' + 'I' * 6,
                                        0x87153284,
                                        0x03180000,
                                        0x31020008,
                                        0x07000707,
                                        0x00301400,
                                        0x2f010000))
        self.publisher_thread = Thread(target=self.publisher_loop)
        self.publisher_thread.start()
        self.sc.run()

    def on_controller_data(self, controller, controller_data):

        if controller_data.status != SCStatus.INPUT:
            return

        previous_data = self._previous_controller_data
        self._previous_controller_data = controller_data

        _xor = previous_data.buttons ^ controller_data.buttons
        buttons = controller_data.buttons
        buttons_lifted = _xor & previous_data.buttons
        buttons_pressed = _xor & controller_data.buttons

        left_grip_down = bool(buttons & SCButtons.LGRIP)
        right_grip_down = bool(buttons & SCButtons.RGRIP)

        if left_grip_down:
            if buttons_pressed & SCButtons.A:
                self.speech_pub.publish("bender")
            if buttons_pressed & SCButtons.B:
                self.speech_pub.publish("rick_and_morty")
            if buttons_pressed & SCButtons.X:
                self.speech_pub.publish("i_am_sorry")
            if buttons_pressed & SCButtons.Y:
                self.speech_pub.publish("take_your_paws")
            if buttons_pressed & SCButtons.BACK:
                self.speech_pub.publish("ultron")
            if buttons_pressed & SCButtons.START:
                self.speech_pub.publish("take_your_paws")
        if right_grip_down:
            if buttons_pressed & SCButtons.A:
                self.move_pub.publish("happy_hand_dance")
            if buttons_pressed & SCButtons.B:
                self.move_pub.publish("happy_dance")
            if buttons_pressed & SCButtons.X:
                self.move_pub.publish("happy_spin")
            if buttons_pressed & SCButtons.Y:
                self.move_pub.publish("sad_emote")
            if buttons_pressed & SCButtons.BACK:
                self.move_pub.publish("wave_hi")
            if buttons_pressed & SCButtons.START:
                self.move_pub.publish("lifted_legs")
        if buttons_pressed & SCButtons.STEAM:
            self.halt_command.publish(HaltCommand(rospy.Time.now(), "Controller comamnd"))
        # # buttons
        # for button in list(SCButtons):
        #     if button & buttons:
        #         if button == SCButtons.LPAD:
        #             pass
        #         # button is down
        #     if button & buttons_pressed:
        #         # button was pressed this event
        #         if button == SCButtons.A:
        #             self.speech_pub.publish("bender")
        #         elif button == SCButtons.B:
        #             self.speech_pub.publish("rick_and_morty")
        #         elif button == SCButtons.X:
        #             self.speech_pub.publish("i_am_sorry")
        #         elif button == SCButtons.Y:
        #             self.speech_pub.publish("ultron")
        #         elif button == SCButtons.RGRIP:
        #             self.speech_pub.publish("take_your_paws")
        #         elif button == SCButtons.RB:
        #             pass
        #     elif button & buttons_lifted:
        #         pass
        #         # button was released this event

        # pads
        robot_x = 0
        robot_y = 0
        robot_rot = 0

        if check_button(buttons, SCButtons.LPADTOUCH):
            x, y = remap_axis(controller_data.lpad_x, controller_data.lpad_y, left_pad=True)
            xp, yp = remap_axis(previous_data.lpad_x, previous_data.lpad_y, left_pad=True)
            self._left_pad_moved += math.sqrt((x - xp)**2 + (y - yp)**2)
            if self._left_pad_moved >= 0.1:
                controller.addFeedback(LEFT_PAD, amplitude=150)
                self._left_pad_moved %= 0.1
            robot_x = -x
            robot_y = y
            # print "Left pad is at X:{0:.3f} Y:{1:.3f}".format(x, y)

        if check_button(buttons, SCButtons.RPADTOUCH):
            x, y = remap_axis(controller_data.rpad_x, controller_data.rpad_y, right_pad=True)
            xp, yp = remap_axis(previous_data.rpad_x, previous_data.rpad_y, right_pad=True)
            self._right_pad_moved += math.sqrt((x - xp)**2 + (y - yp)**2)
            if self._right_pad_moved >= 0.1:
                controller.addFeedback(RIGHT_PAD, amplitude=150)
                self._right_pad_moved %= 0.1
            robot_rot = x
            # print "Right pad is at X:{0:.3f} Y:{1:.3f}".format(x, y)

        if not check_button(buttons, SCButtons.LPADTOUCH) and (controller_data.lpad_x != 0 or controller_data.lpad_y != 0):
            x, y = remap_axis(controller_data.lpad_x, controller_data.lpad_y)
            robot_x = -x
            robot_y = y
            # print "Stick is at X:{0:.3f} Y:{1:.3f}".format(x, y)
        
        lift_height = 2
        cycle_time = 1
        # triggers
        if controller_data.ltrig != 0:
            cycle_time -= 0.75 * scale_trigger(controller_data.ltrig)
            # print "Left trigger at {0:.2f}".format(scale_trigger(controller_data.ltrig))
        if controller_data.rtrig != 0:
            lift_height += 2 * scale_trigger(controller_data.rtrig)
            # print "Right trigger at {0:.2f}".format(scale_trigger(controller_data.rtrig))
        self.update_robot_command(robot_x, robot_y, robot_rot, cycle_time, lift_height=lift_height)

    def publisher_loop(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self._new_command_available:
                self.pub.publish(self._hopper_move_command_msg)
                self._new_command_available = False
            rate.sleep()

    def update_robot_command(self, x, y, rot, cycle_time, lift_height=2):
        move_command = HopperMoveCommand()
        tmp = x
        distance_multiplier = linear_map(cycle_time, 0.25, 1.0, 4.0, 1.0)
        x = y * 0.13 * distance_multiplier
        y = tmp * 0.13 * distance_multiplier
        if abs(rot) > 0.2:
            rot = -rot * 40
        move_command.direction.linear.x = x
        move_command.direction.linear.y = y
        move_command.direction.angular.z = math.radians(rot)
        move_command.lift_height = lift_height
        move_command.cycle_time = cycle_time
        # check if message has changed
        message_changed = (self._hopper_move_command_msg.direction.linear.x != move_command.direction.linear.x or
        self._hopper_move_command_msg.direction.linear.y != move_command.direction.linear.y or
        self._hopper_move_command_msg.direction.angular.z != move_command.direction.angular.z or
        self._hopper_move_command_msg.lift_height != move_command.lift_height or
        self._hopper_move_command_msg.cycle_time != move_command.cycle_time)
        if message_changed:
            self._hopper_move_command_msg = move_command
            self._new_command_available = True

if __name__ == "__main__":
    SteamControllerRosHandler()
