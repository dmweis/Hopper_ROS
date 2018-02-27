from __future__ import division
from __future__ import absolute_import
import math
from enum import IntEnum
from numbers import Number

FEMUR_OFFSET = 13
TIBIA_OFFSET = 35


class Vector3(object):
    def __init__(self, x, y, z):
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

    def rotate_around_z(self, angle):
        old_x = self.x
        old_y = self.y
        angle = math.radians(angle)
        self.x = old_x * math.cos(angle) - old_y * math.sin(angle)
        self.y = old_x * math.sin(angle) + old_y * math.cos(angle)

    def scale(self, number):
        return Vector3(self.x * number, self.y * number, self.z * number)

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2) + math.pow(self.z, 2))

    def move_towards(self, target, distance):
        transform = target - self
        transform_length = transform.length()
        if transform_length < distance:
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
    def __init__(self, x, y, ):
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
    ALL = LEFT_FRONT | RIGHT_FRONT | LEFT_REAR | RIGHT_REAR
    FRONT = LEFT_FRONT | RIGHT_FRONT
    REAR = LEFT_REAR | RIGHT_REAR
    RIGHT = RIGHT_FRONT | RIGHT_REAR
    LEFT = LEFT_FRONT | LEFT_REAR
    RF_LR_CROSS = RIGHT_FRONT | LEFT_REAR
    LF_RR_CROSS = LEFT_FRONT | RIGHT_REAR

    @staticmethod
    def get_legs_as_list(legs):
        selected_legs = []
        if (legs & LegFlags.LEFT_FRONT) != 0:
            selected_legs.append(LegFlags.LEFT_FRONT)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            selected_legs.append(LegFlags.RIGHT_FRONT)
        if (legs & LegFlags.LEFT_REAR) != 0:
            selected_legs.append(LegFlags.LEFT_REAR)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            selected_legs.append(LegFlags.RIGHT_REAR)
        return selected_legs


class LegPositions(object):
    def __init__(self, left_front, right_front, left_rear, right_rear):
        self.left_front = left_front
        self.right_front = right_front
        self.left_rear = left_rear
        self.right_rear = right_rear

    def clone(self):
        return LegPositions(
            self.left_front.clone(),
            self.right_front.clone(),
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
                                self.left_rear - other.left_rear,
                                self.right_rear - other.right_rear)

    def __add__(self, other):
        if isinstance(other, LegPositions):
            return LegPositions(self.left_front + other.left_front,
                                self.right_front + other.right_front,
                                self.left_rear + other.left_rear,
                                self.right_rear + other.right_rear)

    def __truediv__(self, other):
        if isinstance(other, Number):
            return LegPositions(self.left_front / other,
                                self.right_front / other,
                                self.left_rear / other,
                                self.right_rear / other)

    def __mul__(self, other):
        if isinstance(other, Number):
            return LegPositions(self.left_front * other,
                                self.right_front * other,
                                self.left_rear * other,
                                self.right_rear * other)

    def move_towards(self, target, distance):
        LF = self.left_front.move_towards(target.left_front, distance)
        RF = self.right_front.move_towards(target.right_front, distance)
        LR = self.left_rear.move_towards(target.left_rear, distance)
        RR = self.right_rear.move_towards(target.right_rear, distance)
        return LF or RF or LR or RR

    def transform(self, transform, legs = LegFlags.ALL):
        new_position = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            new_position.left_front += transform
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            new_position.right_front += transform
        if (legs & LegFlags.LEFT_REAR) != 0:
            new_position.left_rear += transform
        if (legs & LegFlags.RIGHT_REAR) != 0:
            new_position.right_rear += transform
        return  new_position

    def rotate(self, angle, legs = LegFlags.ALL):
        new_position = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            new_position.left_front.rotate_around_z(angle)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            new_position.right_front.rotate_around_z(angle)
        if (legs & LegFlags.LEFT_REAR) != 0:
            new_position.left_rear.rotate_around_z(angle)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            new_position.right_rear.rotate_around_z(angle)
        return new_position

    def change(self, new_position, legs = LegFlags.ALL):
        position_copy = self.clone()
        if (legs & LegFlags.LEFT_FRONT) != 0:
            position_copy.left_front = new_position
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            position_copy.right_front = new_position
        if (legs & LegFlags.LEFT_REAR) != 0:
            position_copy.left_rear = new_position
        if (legs & LegFlags.RIGHT_REAR) != 0:
            position_copy.right_rear = new_position
        return position_copy

    def get_legs_as_list(self, legs):
        selected_legs = []
        if (legs & LegFlags.LEFT_FRONT) != 0:
            selected_legs.append(self.left_front)
        if (legs & LegFlags.RIGHT_FRONT) != 0:
            selected_legs.append(self.right_front)
        if (legs & LegFlags.LEFT_REAR) != 0:
            selected_legs.append(self.left_rear)
        if (legs & LegFlags.RIGHT_REAR) != 0:
            selected_legs.append(self.right_rear)
        return selected_legs

    def normalize_vectors(self):
        self.left_front = self.left_front / self.left_front.length()
        self.right_front = self.right_front / self.right_front.length()
        self.left_rear = self.left_rear / self.left_rear.length()
        self.right_rear = self.right_rear / self.right_rear.length()

    def __str__(self):
        return 'LF: {} RF: {} LR: {} RR: {}'.format(self.left_front, self.right_front, self.left_rear, self.right_rear)


LEFT_FRONT = LegConfiguration(1, 3, 5, -45, Vector3(6.5, 6.5, 0), -240 + FEMUR_OFFSET, -330 + TIBIA_OFFSET)
RIGHT_FRONT = LegConfiguration(2, 4, 6, 45, Vector3(6.5, -6.5, 0), 60 + FEMUR_OFFSET, -30 + TIBIA_OFFSET)
LEFT_REAR = LegConfiguration(7, 9, 11, -135, Vector3(-6.5, 6.5, 0), 60 + FEMUR_OFFSET, -30 + TIBIA_OFFSET)
RIGHT_REAR = LegConfiguration(8, 10, 12, 135, Vector3(-6.5, -6.5, 0), -240 + FEMUR_OFFSET, -330 + TIBIA_OFFSET)

COXAS = [LEFT_FRONT.coxa_id, RIGHT_FRONT.coxa_id, LEFT_REAR.coxa_id, RIGHT_REAR.coxa_id]
FEMURS = [LEFT_FRONT.femur_id, RIGHT_FRONT.femur_id, LEFT_REAR.femur_id, RIGHT_REAR.femur_id]
TIBIAS = [LEFT_FRONT.tibia_id, RIGHT_FRONT.tibia_id, LEFT_REAR.tibia_id, RIGHT_REAR.tibia_id]

ALL_SERVOS_IDS = COXAS + FEMURS + TIBIAS

COXA_LENGTH = 5.3
FEMUR_LENGTH = 6.5
TIBIA_LENGTH = 13


class IkDriver(object):
    def __init__(self, servo_driver):
        self.__servo_driver = servo_driver

    def setup(self):
        for servo_id in ALL_SERVOS_IDS:
            self.__servo_driver.set_compliance_slope(servo_id, 64)
            self.__servo_driver.set_moving_speed(servo_id, 1023)

    def disable_motors(self):
        for servo_id in ALL_SERVOS_IDS:
            self.__servo_driver.set_torque(servo_id, False)

    def move_legs_synced(self, leg_positions):
        right_front_goal = calculate_ik_for_leg(leg_positions.right_front, RIGHT_FRONT)
        right_rear_goal = calculate_ik_for_leg(leg_positions.right_rear, RIGHT_REAR)
        left_front_goal = calculate_ik_for_leg(leg_positions.left_front, LEFT_FRONT)
        left_rear_goal = calculate_ik_for_leg(leg_positions.left_rear, LEFT_REAR)
        commands = [
            (RIGHT_FRONT.coxa_id, right_front_goal.coxa),
            (RIGHT_FRONT.femur_id, right_front_goal.femur),
            (RIGHT_FRONT.tibia_id, right_front_goal.tibia),
            (RIGHT_REAR.coxa_id, right_rear_goal.coxa),
            (RIGHT_REAR.femur_id, right_rear_goal.femur),
            (RIGHT_REAR.tibia_id, right_rear_goal.tibia),
            (LEFT_FRONT.coxa_id, left_front_goal.coxa),
            (LEFT_FRONT.femur_id, left_front_goal.femur),
            (LEFT_FRONT.tibia_id, left_front_goal.tibia),
            (LEFT_REAR.coxa_id, left_rear_goal.coxa),
            (LEFT_REAR.femur_id, left_rear_goal.femur),
            (LEFT_REAR.tibia_id, left_rear_goal.tibia),
        ]
        self.__servo_driver.group_sync_write_goal_degrees(commands)

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
