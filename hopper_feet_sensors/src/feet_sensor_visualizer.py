#!/usr/bin/env python

import rospy
import tf2_ros

from hopper_feet_sensors.msg import FeetSensorData
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

class FeetSensorVisualizer(object):
    def __init__(self):
        super(FeetSensorVisualizer, self).__init__()
        rospy.init_node("feet_sensor_visualizer")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_publisher = rospy.Publisher("hopper/feet/markers", Marker, queue_size=1)
        rospy.Subscriber("hopper/feet", FeetSensorData, self.on_feet_msg, queue_size=1)
        rospy.spin()

    def on_feet_msg(self, feet_msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.
        marker.scale = Vector3(0.02, 0.02, 0.02)
        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True
        def vector_to_point(vector):
            return Point(vector.x, vector.y, vector.z)
        def lookup_for_foot(link_name, sensor_triggered):
            position = Point()
            try:
                position = self.tf_buffer.lookup_transform(marker.header.frame_id, link_name, 0).transform.translation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            marker.points.append(vector_to_point(position))
            marker.colors.append(ColorRGBA(0, 1, 0, 1) if sensor_triggered else ColorRGBA(1, 0, 0, 1))
        lookup_for_foot("left_front_button", feet_msg.left_front)
        lookup_for_foot("left_middle_button", feet_msg.left_middle)
        lookup_for_foot("left_rear_button", feet_msg.left_rear)
        lookup_for_foot("right_front_button", feet_msg.right_front)
        lookup_for_foot("right_middle_button", feet_msg.right_middle)
        lookup_for_foot("right_rear_button", feet_msg.right_rear)
        self.marker_publisher.publish(marker)

if __name__ == "__main__":
    FeetSensorVisualizer()
