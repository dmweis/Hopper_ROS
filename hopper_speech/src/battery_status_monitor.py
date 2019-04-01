#!/usr/bin/env python

from __future__ import division

import rospy

from std_msgs.msg import String
from hopper_msgs.msg import ServoTelemetrics, HexapodTelemetrics

def mean(numbers):
    return sum(numbers) / max(len(numbers), 1)

SERVO_COUNT = 18


class BatteryStatusMonitor(object):
    critical_voltage_warning_period = rospy.Duration.from_sec(15)

    def __init__(self):
        super(BatteryStatusMonitor, self).__init__()
        rospy.init_node("hopper_battery_monitor", anonymous=True)
        self.lowest_recorded_voltage = 24.0
        self.voltages = {}
        self.first_check = True
        self.speech_publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)
        self.telemetrics_sub = rospy.Subscriber("hopper_telemetrics", HexapodTelemetrics, self.on_new_telemetrics, queue_size=1)
        self.last_critical_voltage_warning = rospy.Time.now() + self.critical_voltage_warning_period
        rospy.spin()

    def on_new_telemetrics(self, message):
        for servo in message.servos:
            self.voltages[servo.id] = servo.voltage
        if len(self.voltages) == SERVO_COUNT:
            voltages = self.voltages.values()
            self.voltages.clear()
            mean_voltage = mean(voltages)
            # skip the first check so that you don't get a warning if battery is already bellow some value
            if self.first_check:
                self.first_check = False
                self.lowest_recorded_voltage = mean_voltage
                return
            if mean_voltage < 10.5 or self.lowest_recorded_voltage < 10.5:
                if self.last_critical_voltage_warning + self.critical_voltage_warning_period < rospy.Time.now():
                    self.speech_publisher.publish("battery_critical")
                    self.last_critical_voltage_warning = rospy.Time.now()
            elif mean_voltage < 11 and self.lowest_recorded_voltage >= 11:
                self.speech_publisher.publish("battery_below_11")
            elif mean_voltage < 12 and self.lowest_recorded_voltage >= 12:
                self.speech_publisher.publish("battery_below_12")
            if mean_voltage < self.lowest_recorded_voltage:
                self.lowest_recorded_voltage = mean_voltage


if __name__ == '__main__':
    BatteryStatusMonitor()
