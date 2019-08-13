#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import paho.mqtt.client as paho
from struct import pack

def progress(x):
    text = '%s messages read' % x
    erase = '\b' * 1000
    print erase,
    print text,

class LaserScanner(object):
    def __init__(self):
        super(LaserScanner, self).__init__()
        self.client = paho.Client()
        self.client.connect("localhost", 1883, 60)
        self.message_counter = 0
        self.biggest_point_count = 0
        self.last_cloud = None

        rospy.init_node("pointcloud_to_csv")
        rospy.Subscriber("/hopper/assembled_scan", PointCloud2, self.on_point_cloud, queue_size=1)
        rospy.spin()

    def on_point_cloud(self, msg):
        payload = [pack(">fff",x, y, z) for (x, y, z) in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))]
        payload = "".join(payload)
        self.client.publish("cloudyboiz", payload)


if __name__ == "__main__":
    LaserScanner()
