
DYNAMIXEL_ERROR_CODES = {
    0: 'COMM_SUCCESS - tx or rx packet communication success',
    -1000: 'COMM_PORT_BUSY - Port is busy (in use)',
    -1001: 'COMM_TX_FAIL - Failed transmit instruction packet',
    -1002: 'COMM_RX_FAIL - Failed get status packet',
    -2000: 'COMM_TX_ERROR - Incorrect instruction packet',
    -3000: 'COMM_RX_WAITING - Now receiving status packet',
    -3001: 'COMM_RX_TIMEOUT - There is no status packet',
    -3002: 'COMM_RX_CORRUPT - Incorrect status packet',
    -9000: 'COMM_NOT_AVAILABLE - Protocol doesn\'t support this feature'
}


def get_tx_rx_result_description(error_code):
    return DYNAMIXEL_ERROR_CODES.get(error_code, "Unknown error")


ERRBIT_VOLTAGE = 1      # Supplied voltage is out of the range (operating voltage set in the control table)
ERRBIT_ANGLE = 2      # Goal position is written out of the range (from CW angle limit to CCW angle limit)
ERRBIT_OVERHEAT = 4      # Temperature is out of the range (operating temperature set in the control table)
ERRBIT_RANGE = 8      # Command(setting value) is out of the range for use.
ERRBIT_CHECKSUM = 16      # Instruction packet checksum is incorrect.
ERRBIT_OVERLOAD = 32      # The current load cannot be controlled by the set torque.
ERRBIT_INSTRUCTION = 64      # Undefined instruction or delivering the action command without the reg_write command.


def get_rx_pack_error_description(error):
    if (error & ERRBIT_VOLTAGE) != 0:
        return '[RxPacketError] Input voltage error!'
    if (error & ERRBIT_ANGLE) != 0:
        return '[RxPacketError] Angle limit error!'
    if (error & ERRBIT_OVERHEAT) != 0:
        return '[RxPacketError] Overheat error!'
    if (error & ERRBIT_RANGE) != 0:
        return '[RxPacketError] Out of range error!'
    if (error & ERRBIT_CHECKSUM) != 0:
        return '[RxPacketError] Checksum error!'
    if (error & ERRBIT_OVERLOAD) != 0:
        return '[RxPacketError] Overload error!'
    if (error & ERRBIT_INSTRUCTION) != 0:
        return '[RxPacketError] Instruction code error!'
    return "Unknown error"
