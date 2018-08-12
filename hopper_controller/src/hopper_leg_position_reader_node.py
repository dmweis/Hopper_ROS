#!/usr/bin/env python

import rospy
from ros_abstraction import IkController


class LegPositionReader(object):
    def __init__(self):
        super(LegPositionReader, self).__init__()
        rospy.init_node("hopper_leg_position_reader")
        ik_controller = IkController()
        ik_controller.disable_motors()
        loop_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ik_controller.read_current_leg_positions()
            loop_rate.sleep()


if __name__ == "__main__":
    LegPositionReader()
