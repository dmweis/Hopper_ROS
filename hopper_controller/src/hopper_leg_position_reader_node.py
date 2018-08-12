#!/usr/bin/env python

import rospy
from ros_abstraction import IkController
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

def create_marker_for_feet(leg_positions):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.get_time()
    marker.ns = ""
    marker.id = 0
    marker.type = Marker.points
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.
    marker.scale = Vector3(0.1, 0.1, 0.1)
    marker.color.a = 1
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.lifetime = 0
    marker.frame_locked = True

    def vector_to_point(vector):
        return Point(vector.x, vector.y, vector.z)
    marker.points.append(vector_to_point(leg_positions.left_front))
    marker.points.append(vector_to_point(leg_positions.right_front))
    marker.points.append(vector_to_point(leg_positions.left_middle))
    marker.points.append(vector_to_point(leg_positions.right_middle))
    marker.points.append(vector_to_point(leg_positions.left_rear))
    marker.points.append(vector_to_point(leg_positions.right_rear))
    return marker


class LegPositionReader(object):
    def __init__(self):
        super(LegPositionReader, self).__init__()
        rospy.init_node("hopper_leg_position_reader")
        marker_publisher = rospy.Publisher("hopper/feet/markers", Marker, queue_size=10)
        ik_controller = IkController()
        ik_controller.disable_motors()
        loop_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            leg_positions = ik_controller.read_current_leg_positions()
            markers = create_marker_for_feet(leg_positions)
            marker_publisher.publish(markers)
            loop_rate.sleep()


if __name__ == "__main__":
    LegPositionReader()
