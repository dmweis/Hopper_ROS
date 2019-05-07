#!/usr/bin/env python

from __future__ import division

import rospy

from math import cos, sin, pi, radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32
from laser_assembler.srv import AssembleScans2, AssembleScans2Request
from hopper_laser.srv import HopperScan, HopperScanResponse

class LaserScanner(object):
    def __init__(self):
        super(LaserScanner, self).__init__()
        rospy.init_node("hopper_laser_scanner")
        rospy.wait_for_service("assemble_scans2")
        self.scanner_active = rospy.get_param("laser_scanner_active_startup", default=False)
        self.scan_time = rospy.get_param("laser_scanner_scan_time", default=5.0)
        self.stance_translate = rospy.Publisher('hopper/stance_translate', Twist, queue_size=1)
        self.point_cloud_publisher = rospy.Publisher("hopper/assembled_scan", PointCloud2, queue_size=2)
        rospy.Subscriber("hopper/stance_translate", Twist, self.on_stance_msg, queue_size=10)
        rospy.Subscriber("hopper/laser_scanner/active", Bool, self.on_scanner_active_msg, queue_size=10)
        rospy.Subscriber("hopper/laser_scanner/scan_time", Float32, self.on_scanner_time, queue_size=10)
        self.laser_scanner_service = rospy.Service("hopper/laser_scanner/scan", HopperScan, self.on_scan_request)
        self.assemble_scan = rospy.ServiceProxy("assemble_scans2", AssembleScans2, persistent=True)
        self.robot_pose = Twist()
        last_pointcloud_time = rospy.get_time()
        pointcloud_timeout = 0.5
        max_tilt_angle = radians(12.0)
        scan_start = rospy.get_time()
        update_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.scanner_active:
                rospy.sleep(0.5)
                continue
            if rospy.get_time() - last_pointcloud_time > pointcloud_timeout:
                last_pointcloud_time = rospy.get_time()
                request = AssembleScans2Request()
                request.begin = rospy.Time.now() - rospy.Duration(self.scan_time)
                request.end = rospy.Time.now()
                point_cloud = self.assemble_scan(request).cloud
                self.point_cloud_publisher.publish(point_cloud)
            if rospy.get_time() - scan_start > self.scan_time:
                scan_start = rospy.get_time()
                max_tilt_angle = -max_tilt_angle
            current_progress = (rospy.get_time() - scan_start) / self.scan_time
            self.robot_pose.angular.y = max_tilt_angle * cos(current_progress * pi)
            self.robot_pose.angular.x = max_tilt_angle * sin(current_progress * pi)
            self.stance_translate.publish(self.robot_pose)
            update_rate.sleep()

    def on_stance_msg(self, msg):
        if msg._connection_header["callerid"] == rospy.get_name():
            return
        self.robot_pose.linear.z = msg.linear.z

    def on_scanner_active_msg(self, msg):
        self.scanner_active = msg.data
        if not self.scanner_active:
            rospy.sleep(0.5)
            self.robot_pose.angular.y = 0
            self.robot_pose.angular.x = 0
            self.stance_translate.publish(self.robot_pose)

    def on_scanner_time(self, msg):
        self.scan_time = msg.data

    def on_scan_request(self, request):
        response = HopperScanResponse()
        if self.scanner_active:
            response.success = False
            return response
        last_pointcloud_time = rospy.get_time()
        pointcloud_timeout = 0.5
        max_tilt_angle = request.vertical_fov if not request.vertical_fov else radians(12.0)
        scan_start = rospy.get_time()
        update_rate = rospy.Rate(30)
        while not rospy.is_shutdown() and not self.scanner_active:
            if rospy.get_time() - last_pointcloud_time > pointcloud_timeout:
                last_pointcloud_time = rospy.get_time()
                assemble_request = AssembleScans2Request()
                assemble_request.begin = rospy.Time.now() - rospy.Duration(request.scan_time)
                assemble_request.end = rospy.Time.now()
                point_cloud = self.assemble_scan(assemble_request).cloud
                self.point_cloud_publisher.publish(point_cloud)
            if rospy.get_time() - scan_start > request.scan_time:
                last_pointcloud_time = rospy.get_time()
                assemble_request = AssembleScans2Request()
                assemble_request.begin = rospy.Time.now() - rospy.Duration(request.scan_time)
                assemble_request.end = rospy.Time.now()
                point_cloud = self.assemble_scan(assemble_request).cloud
                response.cloud = point_cloud
                response.success = True
                self.robot_pose.angular.y = 0
                self.robot_pose.angular.x = 0
                self.stance_translate.publish(self.robot_pose)
                return response
            current_progress = (rospy.get_time() - scan_start) / request.scan_time
            if request.front:
                self.robot_pose.angular.y = max_tilt_angle * cos(current_progress * pi)
            else:
                self.robot_pose.angular.x = max_tilt_angle * cos(current_progress * pi)
            self.stance_translate.publish(self.robot_pose)
            update_rate.sleep()
        response.success = False
        return response


if __name__ == "__main__":
    LaserScanner()
