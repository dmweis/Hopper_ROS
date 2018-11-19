#!/usr/bin/env python

from __future__ import print_function

import serial
import random
import webcolors

from time import sleep


RESET_MSG = 121
RESET_MSG_COUNT = 240
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


class Color():
    def __init__(self, red=0, green=0, blue=0):
        self.red = red
        self.green = green
        self.blue = blue

    def to_data(self):
        return [self.red, self.green, self.blue]

class ColorPacket():
    def __init__(self, from_color=None):
        if from_color is not None:
            self.data = from_color.to_data() * PIXEL_COUNT
        else:
            self.data = Color().to_data() * PIXEL_COUNT

    def set_pixel(self, index, color):
        self.data[index * 3] = color.red
        self.data[index * 3 + 1] = color.green
        self.data[index * 3 + 2] = color.blue

    def to_data(self):
        return bytearray(self.data)


def reset(port):
    payload = bytearray([RESET_MSG] * RESET_MSG_COUNT)
    port.write(payload)

def color_transitions(port, color_a, color_b, steps, delay):
    def get_transitioning_color(val_from, val_to, step, steps):
        val = map_linear(step, 0, steps, val_from, val_to)
        return clamp(int(round(val)), 0, 255)
    for step in range(steps):
        new_color = Color()
        new_color.red = get_transitioning_color(color_a.red, color_b.red, step, steps)
        new_color.green = get_transitioning_color(color_a.green, color_b.green, step, steps)
        new_color.blue = get_transitioning_color(color_a.blue, color_b.blue, step, steps)
        port.write(ColorPacket(new_color).to_data())
        sleep(delay)


def alternate_transitions(port, color_from, color_to, delay):
    current_color = Color(color_from.red, color_from.green, color_from.blue)
    while current_color.red < color_to.red or current_color.green < color_to.green or current_color.blue < color_to.blue:
        if current_color.red < color_to.red:
            current_color.red+=1
        if current_color.green < color_to.green:
            current_color.green+=1
        if current_color.blue < color_to.blue:
            current_color.blue+=1
        port.write(ColorPacket(current_color))
        sleep(delay)
    while current_color.red > color_to.red or current_color.green > color_to.green or current_color.blue > color_to.blue:
        if current_color.red > color_to.red:
            current_color.red-=1
        if current_color.green > color_to.green:
            current_color.green-=1
        if current_color.blue > color_to.blue:
            current_color.blue-=1
        port.write(ColorPacket(current_color))
        sleep(delay)
        

with serial.Serial('/dev/ttyUSB0', 115200) as port:
    reset(port)
    while True:
        # color_transitions(port, Color(20, 0, 0), Color(0, 0, 20), 10, 0.1)
        # color_transitions(port, Color(0, 0, 20), Color(20, 0, 0), 10, 0.1)
        alternate_transitions(port, Color(20, 0, 0), Color(0, 0, 20), 0.01)
        alternate_transitions(port, Color(0, 0, 20), Color(20, 0, 0), 0.01)

