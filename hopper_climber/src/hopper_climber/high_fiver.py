#!/usr/bin/env python

from __future__ import division

import rospy
import numpy as np

from math import radians, ceil, degrees, pi, cos, sin
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from hopper_controller.srv import MoveLegsToPosition, MoveLegsToPositionRequest


def mean(data):
    return float(sum(data)) / max(len(data), 1)

def index_to_heading(index, angle_min, angle_increment):
    return angle_min + index * angle_increment

def heading_to_index(heading, angle_min, angle_increment):
    return max(int(ceil((heading - angle_min) / angle_increment)), 0)


class HighFiveController(object):
    def __init__(self):
        super(HighFiveController, self).__init__()
        rospy.init_node("high_five_controller")
        # variables
        self.active = False
        self.hand_touched = False
        self.left_min_angle_limit = -pi + radians(2)
        self.left_max_angle_limit = -pi + radians(60)
        self.right_min_angle_limit = pi - radians(60)
        self.right_max_angle_limit = pi - radians(2)
        self.max_distance = 0.3
        self.high_five_time = 0.3
        # Subscribers
        self.marker_publisher = rospy.Publisher("hand_marker", Marker, queue_size=10)
        rospy.wait_for_service("hopper/move_limbs_individual")
        rospy.wait_for_service("hopper/move_to_relaxed")
        self.move_legs = rospy.ServiceProxy("hopper/move_limbs_individual", MoveLegsToPosition)
        self.move_legs_to_relaxed = rospy.ServiceProxy("hopper/move_to_relaxed", Empty)
        rospy.Subscriber("hopper/high_five/enabled", Bool, self.on_active_msg, queue_size=10)
        rospy.Subscriber("scan", LaserScan, self.on_laser_msg, queue_size=1)
        rospy.spin()

    def on_laser_msg(self, scan_msg):
        if not self.active:
            return
        left_start_index = heading_to_index(self.left_min_angle_limit, scan_msg.angle_min, scan_msg.angle_increment)
        left_stop_index = heading_to_index(self.left_max_angle_limit, scan_msg.angle_min, scan_msg.angle_increment)
        left_success, x, y = self.detect_hand(left_start_index, left_stop_index, scan_msg)
        if left_success:
            self.display_marker(x, y)
            if not self.hand_touched:
                self.hand_touched = True
                self.touch_point(x, y, True)
                rospy.sleep(self.high_five_time)
                self.move_legs_to_relaxed()
        right_start_index = heading_to_index(self.right_min_angle_limit, scan_msg.angle_min, scan_msg.angle_increment)
        right_stop_index = heading_to_index(self.right_max_angle_limit, scan_msg.angle_min, scan_msg.angle_increment)
        right_success, x, y = self.detect_hand(right_start_index, right_stop_index, scan_msg)
        if right_success:
            self.display_marker(x, y)
            if not self.hand_touched:
                self.hand_touched = True
                self.touch_point(x, y, False)
                rospy.sleep(self.high_five_time)
                self.move_legs_to_relaxed()
        if not left_success and not right_success:
            self.delete_all_markers()
            self.hand_touched = False

    def detect_hand(self, start_index, stop_index, scan_msg):
        detected_points = map(lambda dist: dist < self.max_distance, scan_msg.ranges[start_index:stop_index])
        if True not in detected_points:
            return False, 0, 0
        detected_groups = []
        tmp_start_point = None
        for i in xrange(len(detected_points)):
            if detected_points[i]:
                if tmp_start_point is None:
                    tmp_start_point = i
                    continue
            else:
                if tmp_start_point is not None:
                    detected_groups.append((tmp_start_point, i-1))
                    tmp_start_point = None
        if tmp_start_point is not None:
            detected_groups.append((tmp_start_point, len(detected_points)-1))
        group_distances = map(lambda (start, stop): mean(scan_msg.ranges[start+start_index:stop+start_index]), detected_groups)
        # TODO: less ugly
        largest_group = detected_groups[group_distances.index(min(group_distances))]
        index_middle_largest_group = int((largest_group[0] + largest_group[1]) / 2) + start_index
        heading = index_to_heading(index_middle_largest_group, scan_msg.angle_min, scan_msg.angle_increment)
        distance = scan_msg.ranges[index_middle_largest_group]

        x = distance * cos(heading)
        y = distance * sin(heading)

        return True, x, y

    def touch_point(self, x, y, is_left):
        request = MoveLegsToPositionRequest()
        request.header.frame_id = "laser"
        request.selected_legs = MoveLegsToPositionRequest.LEFT_FRONT if is_left else MoveLegsToPositionRequest.RIGHT_FRONT
        point = Vector3()
        point.x = x
        point.y = y
        point.z = 0
        request.left_front = point
        request.right_front = point
        self.move_legs(request)

    def display_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.x = x
        marker.pose.position.y = y
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

    def delete_all_markers(self):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time()
        marker.action = Marker.DELETEALL
        marker.lifetime = rospy.Duration(0)
        self.marker_publisher.publish(marker)

    def on_active_msg(self, msg):
        self.active = msg.data


if __name__ == "__main__":
    HighFiveController()
