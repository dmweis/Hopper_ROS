from __future__ import absolute_import
import math
import rospy
import tf.transformations as transformations
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from hexapod import Vector3, Vector2


def create_empty_transform_stamped(parent_name, child_name):
    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = parent_name
    transform_stamped.child_frame_id = child_name
    quaternion_zero = transformations.quaternion_from_euler(0, 0, 0)
    transform_stamped.transform.rotation.x = quaternion_zero[0]
    transform_stamped.transform.rotation.y = quaternion_zero[1]
    transform_stamped.transform.rotation.z = quaternion_zero[2]
    transform_stamped.transform.rotation.w = quaternion_zero[3]
    return transform_stamped


def create_empty_odometry_msg(parent_frame_name, child_frame_name):
    odom = Odometry()
    odom.header.frame_id = parent_frame_name
    odom.child_frame_id = child_frame_name
    quaternion_zero = transformations.quaternion_from_euler(0, 0, 0)
    odom.pose.pose.orientation.x = quaternion_zero[0]
    odom.pose.pose.orientation.y = quaternion_zero[1]
    odom.pose.pose.orientation.z = quaternion_zero[2]
    odom.pose.pose.orientation.w = quaternion_zero[3]
    return odom


transform_broadcaster = tf2_ros.TransformBroadcaster()


class OdomPublisher(object):
    def __init__(self, message_publisher, imu_reader, parent_link_name="odom", child_link_name="base_footprint"):
        super(OdomPublisher, self).__init__()
        self.imu_reader = imu_reader
        self._publish_to_tf = rospy.get_param("~publish_odometry_to_tf", True)
        self._transform_broadcaster = transform_broadcaster
        self._odom_publisher = rospy.Publisher('robot_odom', Odometry, queue_size=10)
        self._parent_link_name = parent_link_name
        self._child_link_name = child_link_name
        self.init_direction_known = False
        # initialize default tf transform
        self._last_tf_odometry_message = create_empty_transform_stamped(self._parent_link_name, self._child_link_name)
        self.odometry_rotation = transformations.quaternion_from_euler(0, 0, 0)
        self.odometry_position = Vector2()
        # init odometry message
        self._last_odom_msg = create_empty_odometry_msg(self._parent_link_name, self._child_link_name)
        message_publisher.register_publisher(self)

    def temporary_update_move(self, motion):
        """
        :type motion: Vector2
        """
        yaw = self.imu_reader.get_yaw()
        if yaw is None:
            yaw = 0
        current_orientation = transformations.quaternion_from_euler(0, 0, yaw)
        current_rotation = transformations.euler_from_quaternion(current_orientation)[2]
        temporary_odometry_position = self.odometry_position + motion.rotate_by_angle_rad(current_rotation)
        tf_message = create_empty_transform_stamped(self._parent_link_name, self._child_link_name)
        tf_message.transform.translation.x = temporary_odometry_position.x / 100
        tf_message.transform.translation.y = temporary_odometry_position.y / 100
        tf_message.transform.rotation.x = current_orientation[0]
        tf_message.transform.rotation.y = current_orientation[1]
        tf_message.transform.rotation.z = current_orientation[2]
        tf_message.transform.rotation.w = current_orientation[3]
        odom_message = create_empty_odometry_msg(self._parent_link_name, self._child_link_name)
        odom_message.pose.pose.position.x = temporary_odometry_position.x / 100
        odom_message.pose.pose.position.y = temporary_odometry_position.y / 100
        odom_message.pose.pose.orientation.x = current_orientation[0]
        odom_message.pose.pose.orientation.y = current_orientation[1]
        odom_message.pose.pose.orientation.z = current_orientation[2]
        odom_message.pose.pose.orientation.w = current_orientation[3]
        odom_message.twist.twist.linear.x = self._last_odom_msg.twist.twist.linear.x
        odom_message.twist.twist.linear.y = self._last_odom_msg.twist.twist.linear.y
        odom_message.twist.twist.angular.z = self._last_odom_msg.twist.twist.angular.z
        odom_message.pose.covariance[0] = -1
        self._last_tf_odometry_message = tf_message
        self._last_odom_msg = odom_message

    def update_move(self, motion):
        """
        :type motion: Vector2
        """
        yaw = self.imu_reader.get_yaw()
        if yaw is None:
            yaw = 0
        current_orientation = transformations.quaternion_from_euler(0, 0, yaw)
        current_rotation = transformations.euler_from_quaternion(current_orientation)[2]
        self.odometry_position += motion.rotate_by_angle_rad(current_rotation)
        tf_message = create_empty_transform_stamped(self._parent_link_name, self._child_link_name)
        tf_message.transform.translation.x = self.odometry_position.x / 100
        tf_message.transform.translation.y = self.odometry_position.y / 100
        tf_message.transform.rotation.x = current_orientation[0]
        tf_message.transform.rotation.y = current_orientation[1]
        tf_message.transform.rotation.z = current_orientation[2]
        tf_message.transform.rotation.w = current_orientation[3]
        odom_message = create_empty_odometry_msg(self._parent_link_name, self._child_link_name)
        odom_message.pose.pose.position.x = self.odometry_position.x / 100
        odom_message.pose.pose.position.y = self.odometry_position.y / 100
        odom_message.pose.pose.orientation.x = current_orientation[0]
        odom_message.pose.pose.orientation.y = current_orientation[1]
        odom_message.pose.pose.orientation.z = current_orientation[2]
        odom_message.pose.pose.orientation.w = current_orientation[3]
        odom_message.twist.twist.linear.x = self._last_odom_msg.twist.twist.linear.x
        odom_message.twist.twist.linear.y = self._last_odom_msg.twist.twist.linear.y
        odom_message.twist.twist.angular.z = self._last_odom_msg.twist.twist.angular.z
        odom_message.pose.covariance[0] = -1
        self._last_tf_odometry_message = tf_message
        self._last_odom_msg = odom_message

    def update_translation(self, direction, rotation):
        """
        :type direction: Vector2
        :type rotation: float
        """
        new_rotation = transformations.quaternion_from_euler(0, 0, math.radians(rotation))
        self.odometry_rotation = transformations.quaternion_multiply(new_rotation, self.odometry_rotation)
        current_rotation = transformations.euler_from_quaternion(self.odometry_rotation)[2]
        self.odometry_position += direction.rotate_by_angle_rad(current_rotation)
        tf_message = create_empty_transform_stamped(self._parent_link_name, self._child_link_name)
        tf_message.transform.translation.x = self.odometry_position.x / 100
        tf_message.transform.translation.y = self.odometry_position.y / 100
        tf_message.transform.rotation.x = self.odometry_rotation[0]
        tf_message.transform.rotation.y = self.odometry_rotation[1]
        tf_message.transform.rotation.z = self.odometry_rotation[2]
        tf_message.transform.rotation.w = self.odometry_rotation[3]
        odom_message = create_empty_odometry_msg(self._parent_link_name, self._child_link_name)
        odom_message.pose.pose.position.x = self.odometry_position.x / 100
        odom_message.pose.pose.position.y = self.odometry_position.y / 100
        odom_message.pose.pose.orientation.x = self.odometry_rotation[0]
        odom_message.pose.pose.orientation.y = self.odometry_rotation[1]
        odom_message.pose.pose.orientation.z = self.odometry_rotation[2]
        odom_message.pose.pose.orientation.w = self.odometry_rotation[3]
        odom_message.twist.twist.linear.x = self._last_odom_msg.twist.twist.linear.x
        odom_message.twist.twist.linear.y = self._last_odom_msg.twist.twist.linear.y
        odom_message.twist.twist.angular.z = self._last_odom_msg.twist.twist.angular.z
        self._last_tf_odometry_message = tf_message
        self._last_odom_msg = odom_message

    def update_velocity(self, velocity, theta):
        """
        :param velocity: Vector2 for velocity in x and y axis
        :param theta: rotational speed
        """
        self._last_odom_msg.twist.twist.linear.x = velocity.x / 100
        self._last_odom_msg.twist.twist.linear.y = velocity.y / 100
        self._last_odom_msg.twist.twist.angular.z = math.radians(theta)

    def publish(self):
        if not self.init_direction_known:
            yaw = self.imu_reader.get_yaw()
            if yaw is not None:
                # if yaw is known
                current_orientation = transformations.quaternion_from_euler(0, 0, yaw)
                # set tf message
                self._last_tf_odometry_message.transform.rotation.x = current_orientation[0]
                self._last_tf_odometry_message.transform.rotation.y = current_orientation[1]
                self._last_tf_odometry_message.transform.rotation.z = current_orientation[2]
                self._last_tf_odometry_message.transform.rotation.w = current_orientation[3]
                # set odom msg
                self._last_odom_msg.pose.pose.orientation.x = current_orientation[0]
                self._last_odom_msg.pose.pose.orientation.y = current_orientation[1]
                self._last_odom_msg.pose.pose.orientation.z = current_orientation[2]
                self._last_odom_msg.pose.pose.orientation.w = current_orientation[3]
                self.init_direction_known = True
        now = rospy.Time.now()
        self._last_tf_odometry_message.header.stamp = now
        self._last_odom_msg.header.stamp = now
        if self._publish_to_tf:
            self._transform_broadcaster.sendTransform(self._last_tf_odometry_message)
        self._odom_publisher.publish(self._last_odom_msg)


class HeightPublisher(object):
    def __init__(self, message_publisher, parent_link_name="base_footprint", child_link_name="base_stabilized"):
        super(HeightPublisher, self).__init__()
        self._transform_broadcaster = transform_broadcaster
        self._parent_link_name = parent_link_name
        self._child_link_name = child_link_name
        # initialize default tf transform
        self._last_message = create_empty_transform_stamped(self._parent_link_name, self._child_link_name)
        message_publisher.register_publisher(self)

    def update_height(self, height):
        self._last_message.transform.translation.z = height / 100

    def publish(self):
        self._last_message.header.stamp = rospy.Time.now()
        self._transform_broadcaster.sendTransform(self._last_message)

class BodyOrientationPublisher(object):
    def __init__(self, message_publisher, parent_link_name="base_stabilized", child_link_name="base_link"):
        super(BodyOrientationPublisher, self).__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform_broadcaster = transform_broadcaster
        self.parent_link_name = parent_link_name
        self.child_link_name = child_link_name
        # initialize default tf transform
        self.last_message = create_empty_transform_stamped(self.parent_link_name, self.child_link_name)
        message_publisher.register_publisher(self)

    def publish(self):
        try:
            imu_transform = self.tf_buffer.lookup_transform("base_link", 'imu_plane', rospy.Time())
            euler = transformations.euler_from_quaternion(imu_transform.x, imu_transform.y, imu_transform.z, imu_transform.w)
            rospy.logerr(euler)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        self.last_message.header.stamp = rospy.Time.now()
        self.transform_broadcaster.sendTransform(self._last_message)