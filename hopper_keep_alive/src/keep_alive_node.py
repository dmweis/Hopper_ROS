#!/usr/bin/env python

import rospy
from hopper_keep_alive.srv import Halt, HaltResponse


class KeepAliveNode(object):
    def __init__(self):
        super(KeepAliveNode, self).__init__()
        rospy.init_node("keep_alive_node")
        self.halt_service = rospy.Service("halt", Halt, self.on_halt)
        self.halt_service.spin()

    def on_halt(self, _):
        rospy.loginfo("keep alive node shutting down")
        self.halt_service.shutdown()
        return HaltResponse()

if __name__ == '__main__':
    KeepAliveNode()