#!/usr/bin/env python

import rospy
import psutil
import json

from std_msgs.msg import String


class SystemMonitor(object):
    def __init__(self):
        super(SystemMonitor, self).__init__()
        rospy.init_node("hopper_system_monitor")
        self.core_count = psutil.cpu_count(logical=False)
        self.virtual_core_count = psutil.cpu_count(logical=False)
        self.telemetry_publisher = rospy.Publisher(
            "hopper/system_telemetry", String, queue_size=10, latch=True)
        self.run()

    def run(self):
        # rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            memory_usage = psutil.virtual_memory()
            network_usage = psutil.net_io_counters(pernic=True)
            boot_time = psutil.boot_time()
            cpu_usage = psutil.cpu_percent(interval=1)
            payload = {
                "cpu": {
                    "usage": cpu_usage,
                    "core_count": self.core_count,
                    "virtual_core_count": self.virtual_core_count
                },
                "memory": memory_usage,
                "network": network_usage,
                "boot_time": boot_time,
            }
            self.telemetry_publisher.publish(json.dumps(payload))
            # rate.sleep()


if __name__ == "__main__":
    SystemMonitor()
