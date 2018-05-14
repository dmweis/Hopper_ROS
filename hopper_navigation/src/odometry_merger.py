#!/usr/bin/env python

from threading import Thread
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped


class OdometryMerger(Thread):
    def __init__(self):
        super(OdometryMerger, self).__init__()
        rospy.init_node("odometry_merger")
        self._merged_message = Odometry()
        self._slam_subscriber = rospy.Subscriber("poseupdate", PoseWithCovarianceStamped, self._on_slam_odometry)
        self._odometry_subscriber = rospy.Subscriber("robot_odom", Odometry, self._on_robot_odometry)
        self._merged_odometry_publisher = rospy.Publisher("odom", Odometry, queue_size=10)
        self.start()

    def _on_slam_odometry(self, slam_odom):
        self._merged_message.header.frame_id = slam_odom.header.frame_id
        self._merged_message.child_frame_id = slam_odom.child_frame_id
        self._merged_message.pose.covariance = slam_odom.pose.covariance
        self._merged_message.pose.pose = slam_odom.pose.pose

    def _on_robot_odometry(self, robot_odom):
        self._merged_message.pose.covariance = robot_odom.twist.covariance
        self._merged_message.twist.twist = robot_odom.pose.twist.twist

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self._merged_message.header.stamp = rospy.Time.now()
            self._merged_odometry_publisher.publish(self._merged_message)
            rate.sleep()

if __name__ == "__main__":
    OdometryMerger()