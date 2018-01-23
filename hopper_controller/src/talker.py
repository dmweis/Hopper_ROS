#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep

def send_command(pub, x, y):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    rospy.loginfo(msg)
    pub.publish(msg)

def talker():
    pub = rospy.Publisher('quadruped_command', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    send_command(pub, 2, 0)
    sleep(4)
    send_command(pub, 0, 0)
    sleep(2)
    send_command(pub, -2, 0)
    sleep(4)
    send_command(pub, 0, 0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

