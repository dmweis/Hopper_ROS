#!/usr/bin/env python

import rospy

from math import radians, degrees
from leap_motion.msg import Human, Hand
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3

def constrain(value, min_value, max_value):
    return min(max_value, max(min_value, value))

def is_open(hand):
    return hand.grab_strength < 0.5

class LeapListener(object):
    def __init__(self):
        super(LeapListener, self).__init__()
        rospy.init_node("leap_listener")
        self.published_zero = True
        self.body_pose_publisher = rospy.Publisher("hopper/stance_translate", Twist, queue_size=10)
        rospy.Subscriber("leap_motion/leap_device", Human, self.on_leap_msg)
        rospy.spin()

    def on_leap_msg(self, msg):
        if msg.right_hand is not None and msg.right_hand.is_present and is_open(msg.right_hand):
            hand = msg.right_hand
            position = Vector3()
            position.x = -constrain(hand.palm_center.z, -0.05, 0.05)
            position.y = -constrain(hand.palm_center.x, -0.05, 0.05)
            position.z = constrain(hand.palm_center.y - 0.15, -0.03, 0.08)
            
            rotation = Vector3()
            # angle_limit = radians(9)
            # rotation.x = -constrain(hand.yaw, -angle_limit, angle_limit)
            # rotation.y = -constrain(hand.roll, -angle_limit, angle_limit)
            # rotation.z = constrain(hand.pitch, -angle_limit, angle_limit)
            
            pose = Twist()
            pose.linear = position
            pose.angular = rotation
            # print pose
            self.body_pose_publisher.publish(pose)
            self.published_zero = False
        elif msg.left_hand is not None and msg.left_hand.is_present and is_open(msg.left_hand):
            hand = msg.left_hand
            position = Vector3()
            position.x = -constrain(hand.palm_center.z, -0.05, 0.05)
            position.y = -constrain(hand.palm_center.x, -0.05, 0.05)
            position.z = constrain(hand.palm_center.y - 0.15, -0.03, 0.08)
            
            rotation = Vector3()
            angle_limit = radians(9)
            rotation.x = -constrain(hand.yaw, -angle_limit, angle_limit)
            rotation.y = -constrain(hand.roll, -angle_limit, angle_limit)
            rotation.z = constrain(hand.pitch, -angle_limit, angle_limit)
            
            pose = Twist()
            pose.linear = position
            pose.angular = rotation
            # print pose
            self.body_pose_publisher.publish(pose)
            self.published_zero = False
        elif not self.published_zero:
            # print "Not moving"
            self.body_pose_publisher.publish(Twist())
            self.published_zero = True


if __name__ == "__main__":
    LeapListener()
