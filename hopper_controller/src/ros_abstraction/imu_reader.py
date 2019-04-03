
import rospy
import numpy

from math import degrees
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf import transformations
from hexapod import Vector3, Vector2


def quat_msg_to_array(msg):
    return numpy.array([msg.x, msg.y, msg.z, msg.w])


class ImuReader(object):
    def __init__(self):
        super(ImuReader, self).__init__()
        self.initial_orientation = Quaternion(0, 0, 0, 1)
        self.orientation = None
        rospy.Subscriber("hopper/imu/data", Imu, self.on_imu_msg, queue_size=10)
        rospy.Subscriber("hopper/imu/zero", Empty, self.on_imu_zero, queue_size=1)

    def on_imu_msg(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = msg.orientation
            return
        current_orientation = transformations.quaternion_multiply(
            quat_msg_to_array(msg.orientation),
            transformations.quaternion_inverse(quat_msg_to_array(self.initial_orientation)))
        data = transformations.euler_from_quaternion(current_orientation)
        self.orientation = Vector3(data[0], data[1], data[2])

    def on_imu_zero(self, _):
        self.initial_orientation = None

    def get_yaw(self):
        if self.orientation is not None:
            return self.orientation.z
