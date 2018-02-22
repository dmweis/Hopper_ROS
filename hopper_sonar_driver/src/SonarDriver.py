#!/usr/bin/env python

import rospy
from hopper_msgs.msg import SonarSweep
import serial.tools.list_ports
import serial
import json

def search_sonar_port():
    ports = list(serial.tools.list_ports.comports())
    dynamixel_port = next(port for port in ports if "USBSerial" in port.description)
    return dynamixel_port.device

class SonarDriver(object):
    def __init__(self):
        self.__port_name = search_sonar_port()
        self.__publisher = rospy.Publisher('sonar_sweep', SonarSweep, queue_size=10)
        rospy.init_node('sonar_driver', anonymous=True)

    def read(self):
        with serial.Serial(self.__port_name, 9600) as port:
            while not rospy.is_shutdown():
                message = port.readline()
                try:
                    data = json.loads(message)
                    sonar_data = SonarSweep()
                    sonar_data.angle = data['angle']
                    sonar_data.leftDistance = data['leftSensor']
                    sonar_data.centerDistance = data['centerSensor']
                    sonar_data.rightDistance = data['rightSensor']
                    self.__publisher.publish(sonar_data)
                except ValueError:
                    rospy.logwarn("Json parse error")

if __name__ == '__main__':
    SonarDriver().read()