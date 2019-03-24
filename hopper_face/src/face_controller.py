#!/usr/bin/env python

from __future__ import print_function, division

import serial
import random
import rospy

from cobs import cobs
from colorsys import hsv_to_rgb, rgb_to_hsv
from rospy import sleep
from std_msgs.msg import String, Bool

PIXEL_COUNT = 40

PIXELS_ON_BIGGER_RING = 24
PIXELS_ON_SMALLER_RING = 16

BIGGER_TOP_INDEX = 7
BIGGER_BOTTOM_INDEX = 19
SMALLER_TOP_INDEX = 35
SMALLER_BOTTOM_INDEX = 27


def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(value, a, b):
    min_value = min(a, b)
    max_value = max(a, b)
    return max(min(max_value, value), min_value)


class Color(object):
    def __init__(self, red=0, green=0, blue=0, name=None):
        super(Color, self).__init__()
        self.red = red
        self.green = green
        self.blue = blue
        if name is not None:
            web_color = webcolors.name_to_rgb(name)
            self.red = web_color.red
            self.green = web_color.green
            self.blue = web_color.blue

    def faded_out(self, reduce_by):
        h, s, v = self.to_hsv()
        v = v - v*reduce_by
        v = clamp(v, 0.0, 1.0)
        return HsvColor(h, s, v)

    def to_data(self):
        return [self.red, self.green, self.blue]

    def to_hsv(self):
        red = map_linear(self.red, 0, 255, 0, 1.0)
        green = map_linear(self.green, 0, 255, 0, 1.0)
        blue = map_linear(self.blue, 0, 255, 0, 1.0)
        return rgb_to_hsv(red, green, blue)

    def clone(self):
        return Color(self.red, self.green, self.blue)


class HsvColor(Color):
    def __init__(self, h, s, v):
        red, green, blue = hsv_to_rgb(h, s, v)
        red = int(red * 255)
        green = int(green * 255)
        blue = int(blue * 255)
        super(HsvColor, self).__init__(red, green, blue)


class ColorPacket():
    def __init__(self, from_color=None):
        if from_color is not None:
            self.data = from_color.to_data() * PIXEL_COUNT
        else:
            self.data = Color().to_data() * PIXEL_COUNT

    def correct_pixel_index(self, index):
        index %= PIXEL_COUNT

        if index >= PIXELS_ON_BIGGER_RING:
            index_on_smaller = index - PIXELS_ON_BIGGER_RING
            final_index = PIXELS_ON_SMALLER_RING - 1 - index_on_smaller + PIXELS_ON_BIGGER_RING
            return final_index
        return index

    def set_pixel(self, index, color):
        index = self.correct_pixel_index(index)
        self.data[index * 3] = color.red
        self.data[index * 3 + 1] = color.green
        self.data[index * 3 + 2] = color.blue

    def get_pixel(self, index):
        index = self.correct_pixel_index(index)
        color = Color()
        color.red = self.data[index * 3]
        color.green = self.data[index * 3 + 1]
        color.blue = self.data[index * 3 + 2]
        return color

    def fade_out(self, reduce_by):
        for index in range(PIXEL_COUNT):
            pixel = self.get_pixel(index).faded_out(reduce_by)
            self.set_pixel(index, pixel)

    def to_data(self):
        return cobs.encode(bytes(bytearray(self.data))) + bytes(bytearray([0]))


COLORS = {
    "red": Color(10, 0, 0),
    "blue": Color(0, 0, 10),
    "green": Color(0, 10, 0),
    "yellow": Color(10, 10, 0),
    "purple": Color(10, 0, 10)
}

BRIGHT_COLORS = {
    "red": Color(180, 0, 0),
    "blue": Color(0, 0, 180),
    "green": Color(0, 180, 0),
    "yellow": Color(100, 100, 0),
    "purple": Color(100, 0, 100)
}


class LedController(object):
    def __init__(self):
        super(LedController, self).__init__()
        rospy.init_node("face_controller")
        self.port = None
        self.selected_mode = ""
        self.selected_color = ""
        self.modes = {}
        rospy.Subscriber("hopper/face/mode", String,
                         self.on_mode_change, queue_size=3)
        # controller ready system
        self.controller_ready = False
        rospy.Subscriber("hopper/main_controller_ready", Bool, self.on_ready_msg, queue_size=2)

    def on_mode_change(self, msg):
        new_mode = msg.data.lower()
        # special case for random
        if new_mode == "random":
            self.selected_color = random.choice(list(COLORS))
            self.selected_mode = random.choice(list(self.modes))
            return
        if ":" in new_mode:
            mode, color = new_mode.split(":")
            if color in COLORS:
                self.selected_color = color
            else:
                rospy.logwarn("Color: " + color + " unknown")
            new_mode = mode
        if new_mode in self.modes:
            self.selected_mode = new_mode
        else:
            rospy.logwarn("Mode: " + new_mode + " unknown")

    def write(self, frame):
        self.port.write(frame.to_data())

    def reset(self):
        self.port.write(ColorPacket().to_data())

    def run(self):
        with serial.Serial('/dev/ttyUSB0', 115200) as port:
            self.port = port
            self.reset()
            self.count_down()
            while not rospy.is_shutdown():
                if not (self.selected_color and self.selected_mode):
                    sleep(0.2)
                    continue 
                selected_mode = self.selected_mode
                for i in self.modes[selected_mode]():
                    if selected_mode != self.selected_mode:
                        break
                    if rospy.is_shutdown():
                        break
                # set all off at the end
            port.write(ColorPacket().to_data())

    def count_down(self):
        for i in range(PIXEL_COUNT):
            data = ColorPacket()
            for pixel in range(i):
                data.set_pixel(pixel, COLORS["blue"])
            self.port.write(data.to_data())
            # at this speed outer circle should take 7 seconds
            # 7 seconds is average boot time
            sleep(0.28)
            if self.controller_ready or rospy.is_shutdown():
                break
        if not self.controller_ready:
            self.port.write(ColorPacket(COLORS["red"]).to_data())
            while not rospy.is_shutdown() and not self.controller_ready:
                sleep(0.2)
        self.port.write(ColorPacket(COLORS["green"]).to_data())
        sleep(1)

    def on_ready_msg(self, msg):
        self.controller_ready = msg.data


class AnimationController(LedController):
    def __init__(self):
        super(AnimationController, self).__init__()
        self.selected_mode = "larson_scanner_2"
        self.selected_color = "purple"
        self.modes = {
            "idle_1": self.idle_1,
            "idle_2": self.idle_2,
            "idle_3": self.idle_3,
            "breathing": self.breathing,
            "fade_out": self.fade_out,
            "larson_scanner": self.larson_scanner,
            "larson_scanner_2": self.larson_scanner_2
        }
        self.run()

    def idle_1(self):
        delay = 0.1
        red = COLORS["red"]
        blue = COLORS["blue"]
        green = COLORS["green"]
        for i in self.alternate_transitions(red, blue, delay):
            yield i
        for i in self.alternate_transitions(blue, green, delay):
            yield i
        for i in self.alternate_transitions(green, red, delay):
            yield i

    def idle_2(self):
        red = COLORS["red"]
        blue = COLORS["blue"]
        for i in self.cycle(red, blue, 0.01):
            yield i
        for i in self.cycle(blue, red, 0.01):
            yield i

    def idle_3(self):
        red = COLORS["red"]
        blue = COLORS["blue"]
        for i in self.color_transitions(red, blue, 10, 0.1):
            yield i
        for i in self.color_transitions(blue, red, 10, 0.1):
            yield i

    def breathing(self):
        for i in self.breathing_animation(lambda:COLORS[self.selected_color], 0.05):
            yield i

    def color_transitions(self, color_a, color_b, steps, delay):
        def get_transitioning_color(val_from, val_to, step, steps):
            val = map_linear(step, 0, steps, val_from, val_to)
            return clamp(int(round(val)), 0, 255)
        for step in range(steps):
            new_color = Color()
            new_color.red = get_transitioning_color(
                color_a.red, color_b.red, step, steps)
            new_color.green = get_transitioning_color(
                color_a.green, color_b.green, step, steps)
            new_color.blue = get_transitioning_color(
                color_a.blue, color_b.blue, step, steps)
            self.port.write(ColorPacket(new_color).to_data())
            yield 0
            sleep(delay)

    def alternate_transitions(self, color_from, color_to, delay):
        current_color = Color(color_from.red, color_from.green, color_from.blue)
        while current_color.red < color_to.red or current_color.green < color_to.green or current_color.blue < color_to.blue:
            if current_color.red < color_to.red:
                current_color.red += 1
            if current_color.green < color_to.green:
                current_color.green += 1
            if current_color.blue < color_to.blue:
                current_color.blue += 1
            self.port.write(ColorPacket(current_color).to_data())
            yield 0
            sleep(delay)
        while current_color.red > color_to.red or current_color.green > color_to.green or current_color.blue > color_to.blue:
            if current_color.red > color_to.red:
                current_color.red -= 1
            if current_color.green > color_to.green:
                current_color.green -= 1
            if current_color.blue > color_to.blue:
                current_color.blue -= 1
            self.port.write(ColorPacket(current_color).to_data())
            yield 0
            sleep(delay)

    def cycle(self, color_from, color_to, delay):
        pixel_data = ColorPacket(color_from)
        for pixel in range(PIXEL_COUNT):
            current_color = Color(
                color_from.red, color_from.green, color_from.blue)
            while current_color.red < color_to.red or current_color.green < color_to.green or current_color.blue < color_to.blue:
                if current_color.red < color_to.red:
                    current_color.red += 1
                if current_color.green < color_to.green:
                    current_color.green += 1
                if current_color.blue < color_to.blue:
                    current_color.blue += 1
                pixel_data.set_pixel(pixel, current_color)
                self.port.write(pixel_data.to_data())
                yield 0
                sleep(delay)
            while current_color.red > color_to.red or current_color.green > color_to.green or current_color.blue > color_to.blue:
                if current_color.red > color_to.red:
                    current_color.red -= 1
                if current_color.green > color_to.green:
                    current_color.green -= 1
                if current_color.blue > color_to.blue:
                    current_color.blue -= 1
                pixel_data.set_pixel(pixel, current_color)
                self.port.write(pixel_data.to_data())
                yield 0
                sleep(delay)

    def breathing_animation(self, color, delay):
        for i in range(15):
            current_color = color()
            new_color = Color()
            new_color.red = 0 if current_color.red == 0 else current_color.red + i
            new_color.green = 0 if current_color.green == 0 else current_color.green + i
            new_color.blue = 0 if current_color.blue == 0 else current_color.blue + i
            pixel_data = ColorPacket(new_color)
            self.port.write(pixel_data.to_data())
            yield 0
            sleep(delay)
        for i in reversed(range(15)):
            current_color = color()
            new_color = Color()
            new_color.red = 0 if current_color.red == 0 else current_color.red + i
            new_color.green = 0 if current_color.green == 0 else current_color.green + i
            new_color.blue = 0 if current_color.blue == 0 else current_color.blue + i
            pixel_data = ColorPacket(new_color)
            self.port.write(pixel_data.to_data())
            yield 0
            sleep(delay)

    def larson_scanner(self):
        for index in range(19, PIXEL_COUNT + 19):
            color = BRIGHT_COLORS[self.selected_color]
            frame = ColorPacket()
            frame.set_pixel(index, color)
            frame.set_pixel(index - 1, color.faded_out(0.6))
            frame.set_pixel(index -2, color.faded_out(0.8))
            frame.set_pixel(index -3, color.faded_out(0.9))
            self.write(frame)
            sleep(0.05)
            yield 0

    def larson_scanner_2(self):
        for index in range(48):
            index_smaller = int(round(map_linear(index, 0, 48, 0, PIXELS_ON_BIGGER_RING)))
            index_bigger = int(round(map_linear(index, 0, 48, PIXELS_ON_BIGGER_RING, PIXELS_ON_BIGGER_RING + PIXELS_ON_SMALLER_RING)))

            color = BRIGHT_COLORS[self.selected_color]
            frame = ColorPacket()
            frame.set_pixel(index_smaller, color)
            frame.set_pixel(index_smaller - 1, color.faded_out(0.6))
            frame.set_pixel(index_smaller -2, color.faded_out(0.8))
            frame.set_pixel(index_smaller -3, color.faded_out(0.9))

            frame.set_pixel(index_bigger, color)
            frame.set_pixel(index_bigger - 1, color.faded_out(0.6))
            frame.set_pixel(index_bigger -2, color.faded_out(0.8))
            frame.set_pixel(index_bigger -3, color.faded_out(0.9))
            self.write(frame)
            sleep(0.05)
            yield 0

    def fade_out(self):
        frame = ColorPacket(COLORS["red"])
        self.write(frame)
        sleep(0.1)
        for index in range(10):
            frame.fade_out(0.1)
            self.write(frame)
            sleep(1)
            yield 0


if __name__ == "__main__":
    AnimationController()
