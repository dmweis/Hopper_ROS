#!/usr/bin/env python

import rospy

from math import radians
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool

def clamp(value, a, b):
    smaller = min(a, b)
    bigger = max(a, b)
    return min(bigger, max(smaller, value))

LIMIT_X = radians(18)
UPPER_LIMIT_Y = radians(20)
LOWER_LIMIT_Y = radians(12)


class FaceFollower(object):
    def __init__(self):
        super(FaceFollower, self).__init__()
        rospy.init_node("face_follower")
        self.current_x = 0
        self.current_y = 0
        self.tracking_enabled = False
        self.stance_publisher = rospy.Publisher("hopper/stance_translate", Twist, queue_size=1)
        rospy.Subscriber("camera/detected_face_position", Pose2D, self.on_new_face_pose, queue_size=1)
        rospy.Subscriber("hopper/face_tracking_enabled", Bool, self.on_tracking_enable_msg, queue_size=10)
        rospy.spin()

    def on_new_face_pose(self, msg):
        if self.tracking_enabled:
            new_x = self.current_x + radians(msg.x * 2)
            new_y = self.current_y + radians(msg.y * 1)
            new_x = clamp(new_x, -LIMIT_X, LIMIT_X)
            new_y = clamp(new_y, LOWER_LIMIT_Y, -UPPER_LIMIT_Y)
            new_twist = Twist()
            new_twist.angular.z = new_x
            new_twist.angular.y = new_y
            self.stance_publisher.publish(new_twist)
            self.current_x = new_x
            self.current_y = new_y

    def on_tracking_enable_msg(self, msg):
        self.tracking_enabled = msg.data
        if not self.tracking_enabled:
            new_twist = Twist()
            new_twist.angular.z = 0
            new_twist.angular.y = 0
            self.stance_publisher.publish(new_twist)
            self.current_x = 0
            self.current_y = 0


if __name__ == "__main__":
    FaceFollower()
