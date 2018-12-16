#!/usr/bin/env python

import rospy

from math import radians
from geometry_msgs.msg import Pose2D, Twist


class FaceFollower(object):
    def __init__(self):
        super(FaceFollower, self).__init__()
        self.last_twist = Twist()
        self.stance_publisher = rospy.Publisher("hopper/stance_translate", Twist, queue_size=1)
        rospy.Subscriber("camera/detected_face_position", Pose2D, self.on_new_face_pose, queue_size=1)
        rospy.spin()

    def on_new_face_pose(self, msg):
        old_x_rot = self.last_twist.angular.z
        old_y_rot = self.last_twist.angular.y
        new_x = old_x_rot + radians(msg.x * 2)
        new_y = old_y_rot + radians(msg.y * 1)
        new_twist = Twist()
        new_twist.angular.z = new_x
        new_twist.angular.y = new_y
        self.stance_publisher.publish(new_twist)
        self.last_twist = new_twist

if __name__ == "__main__":
    FaceFollower()
