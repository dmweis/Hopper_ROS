#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def update_direction(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def listener():
    rospy.init_node('quadruped_controller', anonymous=True)
    rospy.Subscriber("quadruped_command", Twist, update_direction)
    rospy.spin()

if __name__ == '__main__':
    listener()

