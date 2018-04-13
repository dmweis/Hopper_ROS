from __future__ import division
import dynamixel_functions as dynamixel
import serial.tools.list_ports

# Control table address for protocol 1
ID = 3
CW_ANGLE_LIMIT = 6
CCW_ANGLE_LIMIT = 8
TORQUE_ENABLE = 24
LED_ENABLE = 25
CW_COMPLIANCE_MARGIN = 26
CCW_COMPLIANCE_MARGIN = 27
CW_COMPLIANCE_SLOPE = 28
CCW_COMPLIANCE_SLOPE = 29
GOAL_POSITION = 30
MOVING_SPEED = 32
TORQUE_LIMIT = 34
PRESENT_POSITION = 36
PRESENT_SPEED = 38
PRESENT_LOAD = 40
PRESENT_VOLTAGE = 42
PRESENT_TEMP = 43
PRESENT_MOVING = 46

PROTOCOL = 1


def search_usb_2_ax_port():
    ports = list(serial.tools.list_ports.comports())
    dynamixel_port = next(port for port in ports if "USB2AX" in port.description)
    return dynamixel_port.device


def map_linear(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def clamp(value, min_value, max_value):
    return max(min(max_value, value), min_value)


def degrees_to_dynamixel_units(degrees):
    return round(clamp(map_linear(degrees, 0, 300, 0, 1023), 0, 1023))


def dynamixel_units_to_degrees(units):
    return round(clamp(map_linear(units, 0, 1023, 0, 300), 0, 300))


class DynamixelDriver(object):
    def __init__(self, port_name):
        self.__port_num = dynamixel.portHandler(port_name.encode('utf-8'))
        dynamixel.packetHandler()
        if not dynamixel.openPort(self.__port_num):
            raise IOError("Failed to open port")
        if not dynamixel.setBaudRate(self.__port_num, 1000000):
            raise IOError("Failed to set baud rate")

    def ping(self, servo_id):
        dynamixel.ping(self.__port_num, PROTOCOL, servo_id)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.__port_num, 1)
        dxl_error = dynamixel.getLastRxPacketError(self.__port_num, 1)
        if dxl_comm_result != 0:
            return False
        if dxl_error != 0:
            dynamixel.printRxPacketError(PROTOCOL, dxl_error)
            raise IOError()
        return True

    def search_servos(self, start_id=0, end_id=252):
        found_servos = []
        for servo_id in range(start_id, end_id + 1):
            if self.ping(servo_id):
                found_servos.append(servo_id)
        return found_servos

    def set_id(self, servo_id, new_servo_id):
        if self.ping(new_servo_id):
            raise IOError("New Id already taken")
        self.__write_byte(servo_id, ID, new_servo_id)

    def set_led(self, servo_id, status):
        """

        :type servo_id: int
        :type status: bool
        """
        self.__write_byte(servo_id, LED_ENABLE, int(status))

    def set_goal_position(self, servo_id, goal_position):
        self.__write_uint_16(servo_id, GOAL_POSITION, goal_position)

    def set_torque(self, servo_id, value):
        self.__write_byte(servo_id, TORQUE_ENABLE, int(value))

    def read_current_position(self, servo_id):
        return self.__read_uint_16(servo_id, PRESENT_POSITION)

    def read_current_position_degrees(self, servo_id):
        return dynamixel_units_to_degrees(self.read_current_position(servo_id))

    def set_moving_speed(self, servo_id, moving_speed, cw=False):
        if cw:
            moving_speed |= 1 >> 10
        self.__write_uint_16(servo_id, MOVING_SPEED, moving_speed)

    def read_voltage(self, servo_id):
        return self.__read_byte(servo_id, PRESENT_VOLTAGE) / 10.0

    def read_temperature(self, servo_id):
        return self.__read_byte(servo_id, PRESENT_TEMP)

    def read_present_load(self, servo_id):
        load_data = self.__read_uint_16(servo_id, PRESENT_LOAD)
        ccw = bool(load_data & 1 << 10)
        load = (load_data & ~(1 << 10)) / 10
        return -load if ccw else load

    def group_sync_write_goal_degrees(self, commands):
        commands = map(lambda command: (command[0], degrees_to_dynamixel_units(command[1])), commands)
        self.group_sync_write_goal(commands)

    def group_sync_write_goal(self, commands):
        sync_group_num = dynamixel.groupSyncWrite(self.__port_num, PROTOCOL, GOAL_POSITION, 2)
        for servo_id, goal_position in commands:
            success = dynamixel.groupSyncWriteAddParam(sync_group_num, servo_id, int(goal_position), 2)
            if not success:
                raise IOError("failed writing to group sync write")
        dynamixel.groupSyncWriteTxPacket(sync_group_num)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.__port_num, PROTOCOL)
        if dxl_comm_result != 0:
            dynamixel.printTxRxResult(PROTOCOL, dxl_comm_result)
            raise IOError()
        dynamixel.groupSyncWriteClearParam(sync_group_num)

    def set_compliance_slope(self, servo_id, compliance_slope):
        self.__write_byte(servo_id, CW_COMPLIANCE_SLOPE, compliance_slope)
        self.__write_byte(servo_id, CCW_COMPLIANCE_SLOPE, compliance_slope)

    def __write_uint_16(self, servo_id, address, data):
        dynamixel.write2ByteTxRx(self.__port_num, PROTOCOL, servo_id, address, data)
        self.__verify_last_message()

    def __write_byte(self, servo_id, address, data):
        dynamixel.write1ByteTxRx(self.__port_num, PROTOCOL, servo_id, address, data)
        self.__verify_last_message()

    def __read_uint_16(self, servo_id, address):
        incoming = dynamixel.read2ByteTxRx(self.__port_num, PROTOCOL, servo_id, address)
        self.__verify_last_message()
        return incoming

    def __read_byte(self, servo_id, address):
        incoming = dynamixel.read1ByteTxRx(self.__port_num, PROTOCOL, servo_id, address)
        self.__verify_last_message()
        return incoming

    def __verify_last_message(self):
        dxl_comm_result = dynamixel.getLastTxRxResult(self.__port_num, 1)
        dxl_error = dynamixel.getLastRxPacketError(self.__port_num, 1)
        if dxl_comm_result != 0:
            dynamixel.printTxRxResult(PROTOCOL, dxl_comm_result)
            raise IOError()
        if dxl_error != 0:
            dynamixel.printRxPacketError(PROTOCOL, dxl_error)
            raise IOError()

    def close(self):
        dynamixel.closePort(self.__port_num)
