#!/usr/bin/env python

from __future__ import division

import rospy

from math import cos, pi, radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from laser_assembler.srv import AssembleScans2, AssembleScans2Request

class LaserScanner(object):
    def __init__(self):
        super(LaserScanner, self).__init__()
        rospy.init_node("hopper_laser_scanner")
        rospy.wait_for_service("assemble_scans2")
        self.scanner_active = rospy.get_param("laser_scanner_active_startup", default=False)
        self.stance_translate = rospy.Publisher('hopper/stance_translate', Twist, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("hopper/assembled_scan", PointCloud2, queue_size=2)
        rospy.Subscriber("hopper/stance_translate", Twist, self.on_stance_msg, queue_size=10)
        rospy.Subscriber("hopper/laser_scanner/active", Bool, self.on_scanner_active_msg, queue_size=10)
        self.assemble_scan = rospy.ServiceProxy("assemble_scans2", AssembleScans2, persistent=True)
        self.robot_pose = Twist()
        scan_time = 5.0
        max_tilt_angle = radians(12.0)
        scan_start = rospy.get_time()
        update_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.scanner_active:
                rospy.sleep(0.5)
                continue
            if rospy.get_time() - scan_start > scan_time:
                request = AssembleScans2Request()
                request.begin = rospy.Time.now() - rospy.Duration(scan_time)
                request.end = rospy.Time.now()
                point_cloud = self.assemble_scan(request).cloud
                self.point_cloud_publisher.publish(point_cloud)
                scan_start = rospy.get_time()
                max_tilt_angle = -max_tilt_angle
            current_progress = (rospy.get_time() - scan_start) / scan_time
            current_cos = cos(current_progress * pi)
            self.robot_pose.angular.y = max_tilt_angle * current_cos
            self.stance_translate.publish(self.robot_pose)
            update_rate.sleep()

    def on_stance_msg(self, msg):
        if msg._connection_header["callerid"] == rospy.get_name():
            return
        self.robot_pose.linear.z = msg.linear.z

    def on_scanner_active_msg(self, msg):
        self.scanner_active = msg.data


if __name__ == "__main__":
    LaserScanner()
