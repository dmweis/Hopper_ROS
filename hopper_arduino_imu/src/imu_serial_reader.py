#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import serial
import json

def gauss_to_tesla(gauss_value):
    """
    convert from gauss to tesla
    One Tesla is equal to 10^4 gauss
    """
    return 1e4 * gauss_value

def get_vector_from_json(data, field_name, ):
    return Vector3(data[field_name]["x"], data[field_name]["y"], data[field_name]["z"])

def convert_dg_vector_to_rad_vector(vector):
    """
    Convert vector in degrees to radians
    """
    return Vector3(math.radians(vector.x), math.radians(vector.y), math.radians(vector.z))

def convert_gauss_vector_to_tesla_vector(vector):
    """
    Convert vector in gauss to tesla
    """
    return Vector3(gauss_to_tesla(vector.x), gauss_to_tesla(vector.y), gauss_to_tesla(vector.z))

class ImuReader(object):
    def __init__(self):
        rospy.init_node('imu_serial_reader', anonymous=True)
        self._port_name = rospy.get_param("imu_serial_port")
        self._imu_publisher = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self._mag_publisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        self.read()

    def read(self):
        with serial.Serial(self._port_name, 9600) as port:
            while not rospy.is_shutdown():
                message = port.readline()
                try:
                    data = json.loads(message)
                    imu_msg = Imu()
                    imu_msg.header.stamp = rospy.Time.now()
                    imu_msg.header.frame_id = "LSM9DS0"
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = rospy.Time.now()
                    mag_msg.header.frame_id = "LSM9DS0"

                    accel_data = get_vector_from_json(data, "accel")
                    gyro_data = get_vector_from_json(data, "gyro")
                    # convert degrees per second to rad per second
                    gyro_data = convert_dg_vector_to_rad_vector(gyro_data)
                    mag_data = get_vector_from_json(data, "mag")
                    # convert from gauss to tesla
                    mag_data = convert_gauss_vector_to_tesla_vector(mag_data)

                    imu_msg.linear_acceleration = accel_data
                    imu_msg.angular_velocity = gyro_data
                    mag_msg.magnetic_field = mag_data
                    imu_msg.orientation_covariance[0] = -1
                    self._imu_publisher.publish(imu_msg)
                    self._mag_publisher.publish(mag_msg)
                except ValueError:
                    rospy.logwarn("Json parse error")
                except serial.SerialException:
                    rospy.logfatal("Serial port error")

if __name__ == '__main__':
    ImuReader()