#!/usr/bin/env python

import rospy
from hopper_keep_alive.srv import Halt

class KeepAliveNode(object):
    def __init__(self):
        super(KeepAliveNode, self).__init__()
        rospy.init_node("keep_alive_node")
        rospy.Service("halt", Halt, self.on_halt)
        rospy.spin()

    def on_halt(self, _):
        rospy.loginfo("keep alive node shutting down")
        rospy.signal_shutdown("Halt command received")

if __name__ == '__main__':
    KeepAliveNode()