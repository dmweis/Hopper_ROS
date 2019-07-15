from __future__ import division
from __future__ import absolute_import
import math
from enum import IntEnum
from numbers import Number
import rospy
import geometry_msgs.msg
import hopper_controller.msg
from hopper_controller.msg import HexapodMotorPositions, LegMotorPositions


class Vector3(geometry_msgs.msg.Vector3):
    def __init__(self, x=0., y=0., z=0.):
        super(Vector3, self).__init__(x, y, z)

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

    def move_towards_at_speed(self, target, distance):
        """
        Moves vector towards target vector traveling distance each step
        :param target: target vector towards which we are moving
        :param distance: max distance traveled with this step
        :return: returns True if move is not finished
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

    def get_moved_towards_by_portion(self, target, portion):
        """
        calculate position that is moved towards target by portion between 0.0 and 1.0
        :param target: target position
        :param portion: number between 0.0 and 1.0 indicating how far along the line is the desired position
        :return: position along the line
        """
        # if portion is exactly 0.0 or 1.0 return target or start to prevent creation of a new object
        if portion <= 0.0:
            return self
        if portion >= 1.0:
            return target
        transform = target - self
        return self + transform * portion

    def clone(self):
        return Vector3(self.x, self.y, self.z)

    def __str__(self):
        return '<{:.2f} {:.2f} {:.2f}>'.format(self.x, self.y, self.z)

    @staticmethod
    def ros_vector3_to_overload_vector(vector):
        return Vector3(vector.x, vector.y, vector.z)


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

    def __truediv__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x / other.x, self.y / other.y)
        elif isinstance(other, Number):
            return Vector2(self.x / other, self.y / other)

    def __mul__(self, other):
        if isinstance(other, Vector2):
            return Vector2(self.x * other.x, self.y * other.y)
        elif isinstance(other, Number):
            return Vector2(self.x * other, self.y * other)

    def rotate_by_angle_rad(self, angle):
        new_vector = Vector2()
        new_vector.x = math.cos(angle) * self.x - math.sin(angle) * self.y
        new_vector.y = math.sin(angle) * self.x + math.cos(angle) * self.y
        return new_vector

    def length(self):
        return math.sqrt(math.pow(self.x, 2) + math.pow(self.y, 2))

    def is_zero(self):
        return self.x == 0 and self.y == 0

    def clone(self):
        return Vector2(self.x, self.y)

    def __str__(self):
        return '<{:.2f} {:.2f}>'.format(self.x, self.y)


class LegFlags(IntEnum):
    NONE = 0
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


class LegPositions(hopper_controller.msg.HexapodLegPositions):
    def __init__(self, left_front, right_front, left_middle, right_middle, left_rear, right_rear):
        super(LegPositions, self).__init__()
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

    def move_towards_at_speed(self, target, distance):
        LF = self.left_front.move_towards_at_speed(target.left_front, distance)
        RF = self.right_front.move_towards_at_speed(target.right_front, distance)
        LM = self.left_middle.move_towards_at_speed(target.left_middle, distance)
        RM = self.right_middle.move_towards_at_speed(target.right_middle, distance)
        LR = self.left_rear.move_towards_at_speed(target.left_rear, distance)
        RR = self.right_rear.move_towards_at_speed(target.right_rear, distance)
        return LF or RF or LM or RM or LR or RR

    def get_moved_towards_by_portion(self, target, portion):
        """
        Calculate new legs position moved towards a target by a portion of the transform
        :param target: target leg positions
        :param portion: portion of the journey finished
        :return: new leg position on the transform between start and target
        """
        if portion <= 0.0:
            return self
        if portion >= 1.0:
            return target
        new_position = self.clone()
        new_position.left_front = self.left_front.get_moved_towards_by_portion(target.left_front, portion)
        new_position.right_front = self.right_front.get_moved_towards_by_portion(target.right_front, portion)
        new_position.left_middle = self.left_middle.get_moved_towards_by_portion(target.left_middle, portion)
        new_position.right_middle = self.right_middle.get_moved_towards_by_portion(target.right_middle, portion)
        new_position.left_rear = self.left_rear.get_moved_towards_by_portion(target.left_rear, portion)
        new_position.right_rear = self.right_rear.get_moved_towards_by_portion(target.right_rear, portion)
        return new_position

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

    def get_legs_as_list(self, legs=LegFlags.ALL):
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

    def get_center_point(self, legs=LegFlags.ALL):
        all_legs = self.get_legs_as_list(legs)
        legs_count = len(all_legs)
        mean_x = sum(map(lambda leg: leg.x, all_legs)) / legs_count
        mean_y = sum(map(lambda leg: leg.y, all_legs)) / legs_count
        mean_z = sum(map(lambda leg: leg.z, all_legs)) / legs_count
        return Vector3(mean_x, mean_y, mean_z)

    def __str__(self):
        return 'LF: {} RF: {} LM: {} RM: {} LR: {} RR: {}'.format(self.left_front, self.right_front, self.left_middle, self.right_middle, self.left_rear, self.right_rear)

    @staticmethod
    def ros_leg_positions_to_leg_positions(ros_leg_positions):
        leg_position = LegPositions(
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.left_front),
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.right_front),
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.left_middle),
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.right_middle),
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.left_rear),
            Vector3.ros_vector3_to_overload_vector(ros_leg_positions.right_rear)
        )
        return leg_position


class IkDriver(object):
    def __init__(self, body_controller):
        """
        :type body_controller: HexapodBodyController
        """
        self.body_controller = body_controller
        self.coxa_length = rospy.get_param("coxa_length")
        self.femur_length = rospy.get_param("femur_length")
        self.tibia_length = rospy.get_param("tibia_length")
        self.femur_offset = rospy.get_param("femur_offset")
        self.tibia_offset = rospy.get_param("tibia_offset")
        self.legs = rospy.get_param("legs")

    def setup(self):
        self.body_controller.set_motor_compliance(64)
        self.body_controller.set_motor_speed(1023)

    def disable_motors(self):
        self.body_controller.set_torque(False)

    def move_legs_synced(self, leg_positions):
        motor_positions = HexapodMotorPositions()
        motor_positions.left_front = self.calculate_ik_for_leg(leg_positions.left_front, self.legs["left_front"])
        motor_positions.right_front = self.calculate_ik_for_leg(leg_positions.right_front, self.legs["right_front"])
        motor_positions.left_middle = self.calculate_ik_for_leg(leg_positions.left_middle, self.legs["left_middle"])
        motor_positions.right_middle = self.calculate_ik_for_leg(leg_positions.right_middle, self.legs["right_middle"])
        motor_positions.left_rear = self.calculate_ik_for_leg(leg_positions.left_rear, self.legs["left_rear"])
        motor_positions.right_rear = self.calculate_ik_for_leg(leg_positions.right_rear, self.legs["right_rear"])
        self.body_controller.set_motors(motor_positions)

    def read_current_leg_positions(self):
        motor_positions = self.body_controller.read_hexapod_motor_positions()
        return LegPositions(
            self.calculate_fk_for_leg(motor_positions.left_front, self.legs["left_front"]),
            self.calculate_fk_for_leg(motor_positions.right_front, self.legs["right_front"]),
            self.calculate_fk_for_leg(motor_positions.left_middle, self.legs["left_middle"]),
            self.calculate_fk_for_leg(motor_positions.right_middle, self.legs["right_middle"]),
            self.calculate_fk_for_leg(motor_positions.left_rear, self.legs["left_rear"]),
            self.calculate_fk_for_leg(motor_positions.right_rear, self.legs["right_rear"])
        )

    def calculate_ik_for_leg(self, target, leg_config):
        coxa_position = Vector3()
        coxa_position.x = leg_config["position"]["x"]
        coxa_position.y = leg_config["position"]["y"]
        coxa_position.z = leg_config["position"]["z"]
        relative_vector = target - coxa_position
        target_angle = math.degrees(math.atan2(relative_vector.y, relative_vector.x)) + leg_config["angle_offset"]
        horizontal_distance_to_target = math.sqrt(math.pow(relative_vector.x, 2) + math.pow(relative_vector.y, 2))
        horizontal_distance_to_target_without_coxa = horizontal_distance_to_target - self.coxa_length
        absolute_distance_to_target = math.sqrt(math.pow(horizontal_distance_to_target_without_coxa, 2) + math.pow(relative_vector.z, 2))
        # use sss triangle solution to calculate angles
        # use law of cosinus to get angles in two corners
        angle_by_tibia = get_angle_by_a(absolute_distance_to_target, self.femur_length, self.tibia_length)
        angle_by_femur = get_angle_by_a(self.tibia_length, self.femur_length, absolute_distance_to_target)
        # we have angles of the SSS triangle. now we need angle for the servos
        ground_to_target_angle_size = math.degrees(math.atan2(horizontal_distance_to_target_without_coxa, -relative_vector.z))
        if target_angle >= 90 or target_angle <= -90:
            # target is behind me
            # can still happen if target is right bellow me
            raise ArithmeticError("Target angle is " + str(target_angle))
        femur_angle = angle_by_femur + ground_to_target_angle_size
        corrected_femur = math.fabs(leg_config["femur_correction"] + femur_angle)
        corrected_tibia = math.fabs(leg_config["tibia_correction"] + angle_by_tibia)
        corrected_coxa = 150 + target_angle
        return LegMotorPositions(corrected_coxa, corrected_femur, corrected_tibia)

    def calculate_fk_for_leg(self, motor_positions, leg_config):
        femur_angle = abs(motor_positions.femur - abs(leg_config["femur_correction"]))
        tibia_angle = abs(motor_positions.tibia - abs(leg_config["tibia_correction"]))
        coxa_angle = motor_positions.coxa - 150 - leg_config["angle_offset"]
        base_x = math.cos(math.radians(coxa_angle))
        base_y = math.sin(math.radians(coxa_angle))
        coxa_vector = Vector3(base_x, base_y, 0) * self.coxa_length
        femur_x = math.sin(math.radians(femur_angle - 90)) * self.femur_length
        femur_y = math.cos(math.radians(femur_angle - 90)) * self.femur_length
        femur_vector = Vector3(base_x * femur_y, base_y * femur_y, femur_x)
        # to calculate tibia we need angle between tibia and a vertical line
        # we get this by calculating the angles formed by a horizontal line from femur
        # femur and part of tibia by knowing that the sum of angles is 180
        # than we just remove this from teh tibia angle and done
        angle_for_tibia_vector = tibia_angle - (180 - 90 - (femur_angle - 90))
        tibia_x = math.sin(math.radians(angle_for_tibia_vector)) * self.tibia_length
        tibia_y = math.cos(math.radians(angle_for_tibia_vector)) * self.tibia_length
        tibia_vector = Vector3(base_x * tibia_x, base_y * tibia_x, -tibia_y)
        coxa_position = Vector3()
        coxa_position.x = leg_config["position"]["x"]
        coxa_position.y = leg_config["position"]["y"]
        coxa_position.z = leg_config["position"]["z"]
        return coxa_position + coxa_vector + femur_vector + tibia_vector


def get_angle_by_a(a, b, c):
    upper = math.pow(b, 2) + math.pow(c, 2) - math.pow(a, 2)
    bottom = 2 * b * c
    divident = max(min(1.0, upper / bottom), -1.0)
    return math.degrees(math.acos(divident))
