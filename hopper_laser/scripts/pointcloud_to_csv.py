#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def progress(x):
    text = '%s messages read' % x
    erase = '\b' * 1000
    print erase,
    print text,

class LaserScanner(object):
    def __init__(self):
        super(LaserScanner, self).__init__()

        self.message_counter = 0
        self.biggest_point_count = 0
        self.last_cloud = None

        rospy.init_node("pointcloud_to_csv")
        rospy.Subscriber("/hopper/assembled_scan", PointCloud2, self.on_point_cloud, queue_size=1)
        raw_input("Press enter to save message\n")
        with open('test_output.csv', 'w') as file:
            for point in pc2.read_points(self.last_cloud, skip_nans=True, field_names=("x", "y", "z")):
                # print point
                file.write(str(point).replace("(", "").replace(")", "") + "\n")
        print "Saved last message"

    def on_point_cloud(self, msg):
        # self.message_counter+=1
        count = sum(1 for _ in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        if count > self.biggest_point_count:
            self.biggest_point_count = count
            self.last_cloud = msg
            print "Found biggest", count
        # progress(self.message_counter)


if __name__ == "__main__":
    LaserScanner()
