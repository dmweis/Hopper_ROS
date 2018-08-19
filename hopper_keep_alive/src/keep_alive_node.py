#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty


class KeepAliveNode(object):
    def __init__(self):
        super(KeepAliveNode, self).__init__()
        rospy.init_node("keep_alive_node")
        self.keep_alive = True
        rospy.Subscriber("halt", Empty, self.on_halt)
        while not rospy.is_shutdown() and self.keep_alive:
            rospy.sleep(1)

    def on_halt(self, _):
        rospy.loginfo("keep alive node shutting down")
        self.keep_alive = False


if __name__ == '__main__':
    KeepAliveNode()
