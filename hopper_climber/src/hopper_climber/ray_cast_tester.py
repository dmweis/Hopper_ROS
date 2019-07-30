#!/usr/bin/env python
from __future__ import division

import rospy

from hopper_obstacle_detector.srv import CastRay, CastRayRequest
from geometry_msgs.msg import Vector3, Vector3Stamped
from visualization_msgs.msg import Marker


class RayCastTester(object):
    def __init__(self):
        super(RayCastTester, self).__init__()
        rospy.init_node("blind_climb_controller")
        rospy.wait_for_service("hopper/detected_obstacles/cast_ray")
        self.cast_ray = rospy.ServiceProxy("hopper/detected_obstacles/cast_ray", CastRay)
        self.marker_publisher = rospy.Publisher("hopper/detected_obstacles/cast_ray/markers", Marker, queue_size=10, latch=True)

        request = CastRayRequest()
        direction = Vector3(1, 0, 0)
        origin = Vector3Stamped()
        origin.header.frame_id = "body_link"
        origin.vector = Vector3(0, 0, 0)
        request.origins.append(origin)
        request.directions.append(direction)
        response = self.cast_ray(request)
        
        for i in range(len(response.hits)):
            if response.hits[i]:
                self.display_marker(response.targets[i].vector.x, response.targets[i].vector.y, response.targets[i].vector.z)
        rospy.spin()

    def display_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "body_link"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.SPHERE
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
