#!/usr/bin/env python
from __future__ import division

import rospy

from hopper_controller.srv import ReadCurrentLegPositions, ReadCurrentLegPositionsRequest
from visualization_msgs.msg import Marker


class RayCastTester(object):
    def __init__(self):
        super(RayCastTester, self).__init__()
        rospy.init_node("blind_climb_controller")
        rospy.wait_for_service("hopper/read_current_leg_positions")
        self.cast_ray = rospy.ServiceProxy("hopper/read_current_leg_positions", ReadCurrentLegPositions)
        self.marker_publisher = rospy.Publisher("hopper/current_leg_positions/markers", Marker, queue_size=10, latch=True)

        request = ReadCurrentLegPositionsRequest()
        request.header.frame_id = "base_footprint"
        response = self.cast_ray(request)
        
        print response
        self.display_marker(response.left_front.x, response.left_front.y, response.left_front.z)
        rospy.spin()

    def display_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        self.marker_publisher.publish(marker)

if __name__ == "__main__":
    RayCastTester()
