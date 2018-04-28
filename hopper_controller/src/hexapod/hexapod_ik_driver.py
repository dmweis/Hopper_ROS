from __future__ import division
from __future__ import absolute_import
import math
from enum import IntEnum
from numbers import Number

FEMUR_OFFSET = 13
TIBIA_OFFSET = 35


class Vector3(object):
    def __init__(self, x=0., y=0., z=0.):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)
        elif isinstance(other, Number):
            return Vector3(self.x + other, self.y + other, self.z + other)

    def __sub__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)
        elif isinstance(other, Number):
            return Vector3(self.x - other, self.y - other, self.z - other)

    def __mul__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x * other.x, self.y * other.y, self.z * other.z)
        elif isinstance(other, Number):
            return Vector3(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other):
        if isinstance(other, Vector3):
            return Vector3(self.x / other.x, self.y / other.y, self.z / other.z)
        elif isinstance(other, Number):
            return Vector3(self.x / other, self.y / other, self.z / other)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.x == other.x and \
                   self.y == other.y and \
                   self.z == other.z
        return False

    def rotate_euler(self, rotation):
        self.rotate_around_x(rotation.x)
        self.rotate_around_y(rotation.y)
        self.rotate_around_z(rotation.z)

    def rotate_around_z(self, angle):
        old_x = self.x
        old_y = self.y
        angle = math.radians(angle)
        self.x = old_x * math.cos(angle) - old_y * math.sin(angle)
        self.y = old_x * math.sin(angle) + old_y * math.cos(angle)

    def rotate_around_x(self, angle):
        old_z = self.z
        old_y = self.y
        angle = math.radians(angle)
        self.z = old_z * math.cos(angle) - old_y * math.sin(angle)
        self.y = old_z * math.sin(angle) + old_y * math.cos(angle)

    def rotate_around_y(self, angle):
        old_x = self.x
        old_z = self.z
        angle = math.radians(angle)
        self.x = old_x * math.cos(angle) - old_z * math.sin(angle)
        self.z = old_x * math.sin(angle) + old_z * math.cos(angle)

    def scale(self, number):
        return Vector3(self.x * number, self.y * number, self.z * number)

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2) + math.pow(self.z, 2))

    def normalize(self):
        if self.length() != 0:
            return self.clone() / self.length()
        return self.clone()

    def move_towards(self, target, distance):
        """
        Moves vector towards target vector traveling distance each step
        :param target: target vector towards which we are moving
        :param distance: max distance traveled with this step
        :return: returns True is move is not finished
        """
        transform = target - self
        transform_length = transform.length()
        # if no distance to travel just finish
        if transform_length * distance == 0:
            return False
        if transform_length <= distance:
            self.x = target.x
            self.y = target.y
            self.z = target.z
            return False
        new_position = self + transform / transform_length * distance
        self.x = new_position.x
        self.y = new_position.y
        self.z = new_position.z
        return True

    def clone(self):
        return Vector3(self.x, self.y, self.z)

    def __str__(self):
        return '<{:.2f} {:.2f} {:.2f}>'.format(self.x, self.y, self.z)


class Vector2(object):
    def __init__(self, x=0., y=0.):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.x == other.x and \
                   self.y == other.y
        return False

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def is_zero(self):
        return self.x == 0 and self.y == 0

    def clone(self):
        return Vector2(self.x, self.y)

    def __str__(self):
        return '<{:.2f} {:.2f}>'.format(self.x, self.y)


class LegConfiguration(object):
    def __init__(self, coxa_id, femur_id, tibia_id, angle_offset, coxa_position, femur_correction, tibia_correction):
        self.coxa_id = coxa_id
        self.femur_id = femur_id
        self.tibia_id = tibia_id
        self.angle_offset = angle_offset
        self.coxa_position = coxa_position
        self.femur_correction = femur_correction
        self.tibia_correction = tibia_correction


class MotorGoalPositions(object):
    def __init__(self, coxa, femur, tibia):
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia


class LegFlags(IntEnum):
    LEFT_FRONT = 1
    RIGHT_FRONT = 2
    LEFT_REAR = 4
    RIGHT_REAR = 8
    LEFT_MIDDLE = 16
    RIGHT_MIDDLE = 32
    ALL = LEFT_FRONT | RIGHT_FRONT | LEFT_REAR | RIGHT_REAR | LEFT_MIDDLE | RIGHT_MIDDLE
    FRONT = LEFT_FRONT | RIGHT_FRONT
    REAR = LEFT_REAR | RIGHT_REAR
    MIDDLE = LEFT_MIDDLE | RIGHT_MIDDLE
    RIGHT = RIGHT_FRONT | RIGHT_REAR | RIGHT_MIDDLE
    LEFT = LEFT_FRONT | LEFT_REAR | LEFT_MIDDLE
    RF_LR_CROSS = RIGHT_FRONT | LEFT_REAR
    LF_RR_CROSS = LEFT_FRONT | RIGHT_REAR
    LEFT_TRIPOD = LEFT_FRONT | RIGHT_MIDDLE | LEFT_REAR
    RIGHT_TRIPOD = RIGHT_FRONT | LEFT_MIDDLE | RIGHT_REAR

    @staticmethod
    def get_legs_as_list(legs):
        selected_legs = []
        if (legs & LegFlags.LEFT_FRONT) != 0:
            selected_legs.append(LegFlags.LEFT_FRONT)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            selected_legs.append(LegFlags.RIGHT_FRONT)
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            selected_legs.append(LegFlags.LEFT_MIDDLE)
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            selected_legs.append(LegFlags.RIGHT_MIDDLE)
        if (legs & LegFlags.LEFT_REAR) != 0:
            selected_legs.append(LegFlags.LEFT_REAR)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            selected_legs.append(LegFlags.RIGHT_REAR)
        return selected_legs


class LegPositions(object):
    def __init__(self, left_front, right_front, left_middle, right_middle, left_rear, right_rear):
        self.left_front = left_front
        self.right_front = right_front
        self.left_middle = left_middle
        self.right_middle = right_middle
        self.left_rear = left_rear
        self.right_rear = right_rear

    def clone(self):
        return LegPositions(
            self.left_front.clone(),
            self.right_front.clone(),
            self.left_middle.clone(),
            self.right_middle.clone(),
            self.left_rear.clone(),
            self.right_rear.clone(),
        )

    def __eq__(self, other):
        if isinstance(self, other.__class__):
            return self.__dict__ == other.__dict__
        return False

    def __sub__(self, other):
        if isinstance(other, LegPositions):
            return LegPositions(self.left_front - other.left_front,
                                self.right_front - other.right_front,
                                self.left_middle - other.left_middle,
                                self.right_middle - other.right_middle,
                                self.left_rear - other.left_rear,
                                self.right_rear - other.right_rear)

    def __add__(self, other):
        if isinstance(other, LegPositions):
            return LegPositions(self.left_front + other.left_front,
                                self.right_front + other.right_front,
                                self.left_middle + other.left_middle,
                                self.right_middle + other.right_middle,
                                self.left_rear + other.left_rear,
                                self.right_rear + other.right_rear)

    def __truediv__(self, other):
        if isinstance(other, Number):
            return LegPositions(self.left_front / other,
                                self.right_front / other,
                                self.left_middle / other,
                                self.right_middle / other,
                                self.left_rear / other,
                                self.right_rear / other)

    def __mul__(self, other):
        if isinstance(other, Number):
            return LegPositions(self.left_front * other,
                                self.right_front * other,
                                self.left_middle * other,
                                self.right_middle * other,
                                self.left_rear * other,
                                self.right_rear * other)

    def move_towards(self, target, distance):
        LF = self.left_front.move_towards(target.left_front, distance)
        RF = self.right_front.move_towards(target.right_front, distance)
        LM = self.left_middle.move_towards(target.left_middle, distance)
        RM = self.right_middle.move_towards(target.right_middle, distance)
        LR = self.left_rear.move_towards(target.left_rear, distance)
        RR = self.right_rear.move_towards(target.right_rear, distance)
        return LF or RF or LM or RM or LR or RR

    def transform(self, transform, legs=LegFlags.ALL):
        new_position = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            new_position.left_front += transform
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            new_position.right_front += transform
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            new_position.left_middle += transform
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            new_position.right_middle += transform
        if (legs & LegFlags.LEFT_REAR) != 0:
            new_position.left_rear += transform
        if (legs & LegFlags.RIGHT_REAR) != 0:
            new_position.right_rear += transform
        return new_position

    def turn(self, angle, legs=LegFlags.ALL):
        new_position = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            new_position.left_front.rotate_around_z(angle)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            new_position.right_front.rotate_around_z(angle)
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            new_position.left_middle.rotate_around_z(angle)
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            new_position.right_middle.rotate_around_z(angle)
        if (legs & LegFlags.LEFT_REAR) != 0:
            new_position.left_rear.rotate_around_z(angle)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            new_position.right_rear.rotate_around_z(angle)
        return new_position

    def rotate(self, euler, legs=LegFlags.ALL):
        new_position = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            new_position.left_front.rotate_euler(euler)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            new_position.right_front.rotate_euler(euler)
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            new_position.left_middle.rotate_euler(euler)
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            new_position.right_middle.rotate_euler(euler)
        if (legs & LegFlags.LEFT_REAR) != 0:
            new_position.left_rear.rotate_euler(euler)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            new_position.right_rear.rotate_euler(euler)
        return new_position

    def change(self, new_position, legs=LegFlags.ALL):
        position_copy = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            position_copy.left_front = new_position.clone()
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            position_copy.right_front = new_position.clone()
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            position_copy.left_middle = new_position.clone()
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            position_copy.right_middle = new_position.clone()
        if (legs & LegFlags.LEFT_REAR) != 0:
            position_copy.left_rear = new_position.clone()
        if (legs & LegFlags.RIGHT_REAR) != 0:
            position_copy.right_rear = new_position.clone()
        return position_copy

    def update_from_other(self, other, legs=LegFlags.ALL):
        position_copy = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            position_copy.left_front = other.left_front.clone()
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            position_copy.right_front = other.right_front.clone()
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            position_copy.left_middle = other.left_middle.clone()
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            position_copy.right_middle = other.right_middle.clone()
        if (legs & LegFlags.LEFT_REAR) != 0:
            position_copy.left_rear = other.left_rear.clone()
        if (legs & LegFlags.RIGHT_REAR) != 0:
            position_copy.right_rear = other.right_rear.clone()
        return position_copy

    def get_legs_as_list(self, legs):
        selected_legs = []
        if (legs & LegFlags.LEFT_FRONT) != 0:
            selected_legs.append(self.left_front)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            selected_legs.append(self.right_front)
        if (legs & LegFlags.LEFT_MIDDLE) != 0:
            selected_legs.append(self.left_middle)
        if (legs & LegFlags.RIGHT_MIDDLE) != 0:
            selected_legs.append(self.right_middle)
        if (legs & LegFlags.LEFT_REAR) != 0:
            selected_legs.append(self.left_rear)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            selected_legs.append(self.right_rear)
        return selected_legs

    def normalize_vectors(self):
        self.left_front = self.left_front.normalize()
        self.right_front = self.right_front.normalize()
        self.left_middle = self.left_middle.normalize()
        self.right_middle = self.right_middle.normalize()
        self.left_rear = self.left_rear.normalize()
        self.right_rear = self.right_rear.normalize()

    def longest_length(self):
        LF = self.left_front.length()
        RF = self.right_front.length()
        LM = self.left_middle.length()
        RM = self.right_middle.length()
        LR = self.left_rear.length()
        RR = self.right_rear.length()
        return max(map(abs, [LF, RF, LM, RM, LR, RR]))

    def __str__(self):
        return 'LF: {} RF: {} LM: {} RM: {} LR: {} RR: {}'.format(self.left_front, self.right_front, self.left_middle, self.right_middle, self.left_rear, self.right_rear)


LEFT_FRONT = LegConfiguration(1, 3, 5, -45, Vector3(11.5, 5, 0), -240 + FEMUR_OFFSET, -330 + TIBIA_OFFSET)
RIGHT_FRONT = LegConfiguration(2, 4, 6, 45, Vector3(11.5, -5, 0), 60 + FEMUR_OFFSET, -30 + TIBIA_OFFSET)
LEFT_MIDDLE = LegConfiguration(13, 15, 17, -90, Vector3(0, 10, 0), -240 + FEMUR_OFFSET, -330 + TIBIA_OFFSET)
RIGHT_MIDDLE = LegConfiguration(14, 16, 18, 90, Vector3(0, -10, 0), 60 + FEMUR_OFFSET, -30 + TIBIA_OFFSET)
LEFT_REAR = LegConfiguration(7, 9, 11, -135, Vector3(-11.5, 5, 0), -240 + FEMUR_OFFSET, -330 + TIBIA_OFFSET)
RIGHT_REAR = LegConfiguration(8, 10, 12, 135, Vector3(-11.5, -5, 0), 60 + FEMUR_OFFSET, -30 + TIBIA_OFFSET)

COXAS = [LEFT_FRONT.coxa_id, RIGHT_FRONT.coxa_id, RIGHT_MIDDLE.coxa_id, LEFT_MIDDLE.coxa_id, LEFT_REAR.coxa_id, RIGHT_REAR.coxa_id]
FEMURS = [LEFT_FRONT.femur_id, RIGHT_FRONT.femur_id, RIGHT_MIDDLE.femur_id, LEFT_MIDDLE.femur_id, LEFT_REAR.femur_id, RIGHT_REAR.femur_id]
TIBIAS = [LEFT_FRONT.tibia_id, RIGHT_FRONT.tibia_id, RIGHT_MIDDLE.tibia_id, LEFT_MIDDLE.tibia_id, LEFT_REAR.tibia_id, RIGHT_REAR.tibia_id]

ALL_SERVOS_IDS = COXAS + FEMURS + TIBIAS

COXA_LENGTH = 5.3
FEMUR_LENGTH = 6.5
TIBIA_LENGTH = 13

JOINT_NAMES = [
    "left_front_coxa_joint",
    "left_front_femur_joint",
    "left_front_tibia_joint",
    "right_front_coxa_joint",
    "right_front_femur_joint",
    "right_front_tibia_joint",
    "left_middle_coxa_joint",
    "left_middle_femur_joint",
    "left_middle_tibia_joint",
    "right_middle_coxa_joint",
    "right_middle_femur_joint",
    "right_middle_tibia_joint",
    "left_rear_coxa_joint",
    "left_rear_femur_joint",
    "left_rear_tibia_joint",
    "right_rear_coxa_joint",
    "right_rear_femur_joint",
    "right_rear_tibia_joint",
]


class IkDriver(object):
    def __init__(self, servo_driver, joint_state_publisher):
        """
        :type servo_driver: DynamixelDriver
        :type joint_state_publisher: JointStatePublisher
        """
        self.__servo_driver = servo_driver
        self.joint_state_publisher = joint_state_publisher

    def setup(self):
        for servo_id in ALL_SERVOS_IDS:
            self.__servo_driver.set_compliance_slope(servo_id, 64)
            self.__servo_driver.set_moving_speed(servo_id, 1023)
            self.__servo_driver.set_torque(servo_id, False)

    def disable_motors(self):
        for servo_id in ALL_SERVOS_IDS:
            self.__servo_driver.set_torque(servo_id, False)

    def read_telemetrics(self):
        telemetric_data = []
        for servo_id in ALL_SERVOS_IDS:
            voltage = self.__servo_driver.read_voltage(servo_id)
            temperature = self.__servo_driver.read_temperature(servo_id)
            telemetric_data.append((servo_id, voltage, temperature))
        return telemetric_data

    def move_legs_synced(self, leg_positions):
        right_front_goal = calculate_ik_for_leg(leg_positions.right_front, RIGHT_FRONT)
        right_rear_goal = calculate_ik_for_leg(leg_positions.right_rear, RIGHT_REAR)
        left_middle_goal = calculate_ik_for_leg(leg_positions.left_middle, LEFT_MIDDLE)
        right_middle_goal = calculate_ik_for_leg(leg_positions.right_middle, RIGHT_MIDDLE)
        left_front_goal = calculate_ik_for_leg(leg_positions.left_front, LEFT_FRONT)
        left_rear_goal = calculate_ik_for_leg(leg_positions.left_rear, LEFT_REAR)
        commands = [
            # right front
            (RIGHT_FRONT.coxa_id, right_front_goal.coxa),
            (RIGHT_FRONT.femur_id, right_front_goal.femur),
            (RIGHT_FRONT.tibia_id, right_front_goal.tibia),
            # right rear
            (RIGHT_REAR.coxa_id, right_rear_goal.coxa),
            (RIGHT_REAR.femur_id, right_rear_goal.femur),
            (RIGHT_REAR.tibia_id, right_rear_goal.tibia),
            # left middle
            (LEFT_MIDDLE.coxa_id, left_middle_goal.coxa),
            (LEFT_MIDDLE.femur_id, left_middle_goal.femur),
            (LEFT_MIDDLE.tibia_id, left_middle_goal.tibia),
            # right middle
            (RIGHT_MIDDLE.coxa_id, right_middle_goal.coxa),
            (RIGHT_MIDDLE.femur_id, right_middle_goal.femur),
            (RIGHT_MIDDLE.tibia_id, right_middle_goal.tibia),
            # left front
            (LEFT_FRONT.coxa_id, left_front_goal.coxa),
            (LEFT_FRONT.femur_id, left_front_goal.femur),
            (LEFT_FRONT.tibia_id, left_front_goal.tibia),
            # left rear
            (LEFT_REAR.coxa_id, left_rear_goal.coxa),
            (LEFT_REAR.femur_id, left_rear_goal.femur),
            (LEFT_REAR.tibia_id, left_rear_goal.tibia),
        ]
        self.__servo_driver.group_sync_write_goal_degrees(commands)
        joint_positions = [
            left_front_goal.coxa - 150,
            left_front_goal.femur - 150 + FEMUR_OFFSET,
            left_front_goal.tibia - 150 - TIBIA_OFFSET,
            right_front_goal.coxa - 150,
            right_front_goal.femur - 150 - FEMUR_OFFSET,
            right_front_goal.tibia - 150 + TIBIA_OFFSET,
            left_middle_goal.coxa - 150,
            left_middle_goal.femur - 150 + FEMUR_OFFSET,
            left_middle_goal.tibia - 150 - TIBIA_OFFSET,

            right_middle_goal.coxa - 150,
            right_middle_goal.femur - 150 - FEMUR_OFFSET,
            right_middle_goal.tibia - 150 + TIBIA_OFFSET,
            left_rear_goal.coxa - 150,
            left_rear_goal.femur - 150 + FEMUR_OFFSET,
            left_rear_goal.tibia - 150 - TIBIA_OFFSET,
            right_rear_goal.coxa - 150,
            right_rear_goal.femur - 150 - FEMUR_OFFSET,
            right_rear_goal.tibia - 150 + TIBIA_OFFSET
        ]
        self.joint_state_publisher.update_joint_states(JOINT_NAMES, map(math.radians, joint_positions))

    def read_current_leg_positions(self):
        left_front = self.__read_single_current_leg_position(LEFT_FRONT)
        right_front = self.__read_single_current_leg_position(RIGHT_FRONT)
        left_middle = self.__read_single_current_leg_position(LEFT_MIDDLE)
        right_middle = self.__read_single_current_leg_position(RIGHT_MIDDLE)
        left_rear = self.__read_single_current_leg_position(LEFT_REAR)
        right_rear = self.__read_single_current_leg_position(RIGHT_REAR)
        return LegPositions(left_front,
                            right_front,
                            left_middle,
                            right_middle,
                            left_rear,
                            right_rear)

    def __read_single_current_leg_position(self, leg_configuration):
        motor_positions = MotorGoalPositions(self.__servo_driver.read_current_position_degrees(leg_configuration.coxa_id),
                                  self.__servo_driver.read_current_position_degrees(leg_configuration.femur_id),
                                  self.__servo_driver.read_current_position_degrees(leg_configuration.tibia_id))
        return calculate_fk_for_leg(motor_positions, leg_configuration)

    def close(self):
        self.__servo_driver.close()


def calculate_ik_for_leg(target, leg_config):
    relative_vector = target - leg_config.coxa_position
    target_angle = math.degrees(math.atan2(relative_vector.y, relative_vector.x)) + leg_config.angle_offset
    horizontal_distance_to_target = math.sqrt(math.pow(relative_vector.x, 2) + math.pow(relative_vector.y, 2))
    horizontal_distance_to_target_without_coxa = horizontal_distance_to_target - COXA_LENGTH
    absolute_distance_to_target = math.sqrt(math.pow(horizontal_distance_to_target_without_coxa, 2) + math.pow(relative_vector.z, 2))
    # use sss triangle solution to calculate angles
    # use law of cosinus to get angles in two corners
    angle_by_tibia = get_angle_by_a(absolute_distance_to_target, FEMUR_LENGTH, TIBIA_LENGTH)
    angle_by_femur = get_angle_by_a(TIBIA_LENGTH, FEMUR_LENGTH, absolute_distance_to_target)
    # we have angles of the SSS triangle. now we need angle for the servos
    ground_to_target_angle_size = math.degrees(math.atan2(horizontal_distance_to_target_without_coxa, -relative_vector.z))
    if target_angle >= 90 or target_angle <= -90:
        # target is behind me
        # can still happen if target is right bellow me
        raise ArithmeticError("Target angle is " + str(target_angle))
    femur_angle = angle_by_femur + ground_to_target_angle_size
    corrected_femur = math.fabs(leg_config.femur_correction + femur_angle)
    corrected_tibia = math.fabs(leg_config.tibia_correction + angle_by_tibia)
    corrected_coxa = 150 + target_angle
    return MotorGoalPositions(corrected_coxa, corrected_femur, corrected_tibia)


def get_angle_by_a(a, b, c):
    upper = math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)
    bottom = 2 * b * c
    return math.degrees(math.acos(upper / bottom))


def calculate_fk_for_leg(motor_positions, leg_config):
    femur_angle = abs(motor_positions.femur - abs(leg_config.femur_correction))
    tibia_angle = abs(motor_positions.tibia - abs(leg_config.tibia_correction))
    coxa_angle = motor_positions.coxa - 150 - leg_config.angle_offset
    base_x = math.cos(math.radians(coxa_angle))
    base_y = math.sin(math.radians(coxa_angle))
    coxa_vector = Vector3(base_x, base_y, 0) * COXA_LENGTH
    femur_x = math.sin(math.radians(femur_angle - 90)) * FEMUR_LENGTH
    femur_y = math.cos(math.radians(femur_angle - 90)) * FEMUR_LENGTH
    femur_vector = Vector3(base_x * femur_y, base_y * femur_y, femur_x)
    # to calculate tibia we need angle between tibia and a vertical line
    # we get this by calculating the angles formed by a horizontal line from femur
    # femur and part of fibia by knowing that the sum of angles is 180
    # than we just remove this from teh tibia angle and done
    angle_for_tibia_vector = tibia_angle - (180 - 90 - (femur_angle - 90))
    tibia_x = math.sin(math.radians(angle_for_tibia_vector)) * TIBIA_LENGTH
    tibia_y = math.cos(math.radians(angle_for_tibia_vector)) * TIBIA_LENGTH
    tibia_vector = Vector3(base_x * tibia_x, base_y * tibia_x, -tibia_y)
    return leg_config.coxa_position + coxa_vector + femur_vector + tibia_vector
