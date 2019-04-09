#!/usr/bin/env python

from __future__ import division

import rospy

from std_msgs.msg import String
from hopper_msgs.msg import ServoTelemetry, HexapodTelemetry
from sensor_msgs.msg import BatteryState

def mean(numbers):
    return sum(numbers) / max(len(numbers), 1)

SERVO_COUNT = 18
MAX_VOLTAGE = 12.5
MIN_VOLTAGE = 10.5


class BatteryStatusMonitor(object):
    critical_voltage_warning_period = rospy.Duration.from_sec(15)

    def __init__(self):
        super(BatteryStatusMonitor, self).__init__()
        rospy.init_node("hopper_battery_monitor", anonymous=True)
        self.lowest_recorded_voltage = 24.0
        self.voltages = {}
        self.first_check = True
        self.speech_publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)
        self.battery_publisher = rospy.Publisher("hopper/battery_status", BatteryState, queue_size=5)
        self.telemetry_sub = rospy.Subscriber("hopper_telemetry", HexapodTelemetry, self.on_new_telemetry, queue_size=1)
        self.last_critical_voltage_warning = rospy.Time.now() + self.critical_voltage_warning_period
        rospy.spin()

    def on_new_telemetry(self, message):
        for servo in message.servos:
            self.voltages[servo.id] = servo.voltage
        if len(self.voltages) == SERVO_COUNT:
            voltages = self.voltages.values()
            self.voltages.clear()
            mean_voltage = mean(voltages)
            battery_state = BatteryState()
            battery_state.header.stamp = rospy.Time.now()
            battery_state.voltage = mean_voltage
            battery_state.current = float("nan")
            battery_state.charge = float("nan")
            battery_state.capacity = float("nan")
            battery_state.design_capacity = float("nan")
            battery_state.percentage = 100 - (MAX_VOLTAGE - mean_voltage) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100
            battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            battery_state.present = True
            battery_state.cell_voltage = [float("nan")] * 3
            battery_state.location = "Primary batter bay"
            battery_state.serial_number = "N/A"
            self.battery_publisher.publish(battery_state)
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
