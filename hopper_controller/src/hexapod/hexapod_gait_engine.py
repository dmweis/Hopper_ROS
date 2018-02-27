from __future__ import division
from __future__ import absolute_import
import rospy
from .hexapod_ik_driver import LegPositions, Vector3, Vector2, LegFlags
import threading
from time import sleep, time
import math

INTERPOLATION_FREQUENCY = 30

LEG_HEIGHT = -9
LEG_DISTANCE_LONGITUDAL = 15
LEG_DISTANCE_LATERAL = 15

RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + 5, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - 5, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
)

OFFSET_DISTANCE = 3

GROUND_LEVEL_RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, -3),
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, -3),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + 5 + OFFSET_DISTANCE, -3),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - 5 - OFFSET_DISTANCE, -3),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, -3),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, -3),
)

WIDER_RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + 5 + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - 5 - OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, LEG_HEIGHT),
)


def get_height_for_step(distance, full_step_length, height):
    distance = distance / full_step_length
    return math.sin(distance * math.pi) * height


class GaitController(threading.Thread):
    def __init__(self, ik_driver):
        super(GaitController, self).__init__()
        self.ik_driver = ik_driver
        self.direction = Vector2(0, 0)
        self.rotation = 0
        self.ready = False
        self.__relaxed_transformation = Vector3(0, 0, 0)
        self.__relaxed_rotation = Vector3(0, 0, 0)
        self.__current_relaxed_position = RELAXED_POSITION.clone()
        self.__position_update_ready = False
        self.__last_used_forward_legs = LegFlags.LEFT_TRIPOD
        self.__speed = 9
        self.__update_delay = 1000 / INTERPOLATION_FREQUENCY
        self.__keep_running = True
        self.__relaxed = True
        self.__last_telemetrics_update_time = time()
        self.telemetrics_callback = None
        self.ik_driver.setup()
        self.__last_written_position = self.ik_driver.read_current_leg_positions()
        self.start()

    def run(self):
        try:
            rospy.loginfo("Hexapod gait engine started")
            for leg in LegFlags.get_legs_as_list(LegFlags.ALL):
                new_position = self.__last_written_position.clone().update_from_other(GROUND_LEVEL_RELAXED_POSITION, leg)
                self.__execute_move(new_position, 6)
            self.__execute_move(WIDER_RELAXED_POSITION.clone(), 6)
            self.__go_to_relaxed(self.__get_next_leg_combo(), self.__current_relaxed_position, distance_speed_multiplier=2)
            self.__go_to_relaxed(self.__get_next_leg_combo(), self.__current_relaxed_position, distance_speed_multiplier=2)
            rospy.loginfo("Hexapod ready")
            self.ready = True
            while self.__keep_running:
                if not self.direction.is_zero() or self.rotation != 0:
                    if self.direction.is_zero() and abs(self.rotation) > 8:
                        # just rotation
                        self.__execute_step(self.direction, self.rotation, self.__get_next_leg_combo(), distance_speed_multiplier=6)
                    elif self.direction.length() > 5.5:
                        # fast walking
                        self.__execute_step(self.direction, self.rotation, self.__get_next_leg_combo(), distance_speed_multiplier=5)
                    else:
                        # regular walking
                        self.__execute_step(self.direction, self.rotation, self.__get_next_leg_combo(), distance_speed_multiplier=3)
                    self.__relaxed = False
                elif not self.__relaxed:
                    # move to relaxed
                    self.__go_to_relaxed(self.__get_next_leg_combo(), self.__current_relaxed_position, distance_speed_multiplier=2)
                    if self.direction.is_zero() and self.rotation == 0:
                        self.__go_to_relaxed(self.__get_next_leg_combo(), self.__current_relaxed_position, distance_speed_multiplier=2)
                    self.__relaxed = True
                elif self.__position_update_ready:
                    self.__position_update_ready = False
                    self.update_body_orientation(self.__relaxed_transformation, self.__relaxed_rotation)
                else:
                    sleep(self.__update_delay * 0.001)
                    if self.telemetrics_callback and time() - self.__last_telemetrics_update_time > 4:
                        self.telemetrics_callback(self.ik_driver.read_telemetrics())
                        self.__last_telemetrics_update_time = time()
            self.ready = False
            self.update_body_orientation(Vector3(0, 0, 0), Vector3(0, 0, 0))
            self.__go_to_relaxed(self.__get_next_leg_combo(), WIDER_RELAXED_POSITION, distance_speed_multiplier=2)
            self.__go_to_relaxed(self.__get_next_leg_combo(), WIDER_RELAXED_POSITION, distance_speed_multiplier=2)
            self.__execute_move(GROUND_LEVEL_RELAXED_POSITION.clone(), 3)
        except Exception as e:
            rospy.signal_shutdown("Gait engine loop failed" + str(e))

    def stop(self, disable_motors=True):
        self.__keep_running = False
        self.join()
        if disable_motors:
            self.ik_driver.disable_motors()
        self.ik_driver.close()

    def update_relaxed_position(self, transform=None, rotation=None):
        if transform is not None:
            self.__relaxed_transformation = transform
        if rotation is not None:
            self.__relaxed_rotation = rotation
        self.__position_update_ready = True

    def __execute_step(self, direction, angle, forward_legs, speed_override=None, distance_speed_multiplier=None, leg_lift_height=2):
        backwards_legs = LegFlags.RIGHT_TRIPOD if forward_legs == LegFlags.LEFT_TRIPOD else LegFlags.LEFT_TRIPOD
        start_position = self.__last_written_position.clone()
        target_position = self.__current_relaxed_position.clone() \
            .transform(Vector3(direction.x / 2, direction.y / 2, 0), forward_legs) \
            .turn(-angle / 2, forward_legs) \
            .transform(Vector3(-direction.x / 2, -direction.y / 2, 0), backwards_legs) \
            .turn(angle / 2, backwards_legs)
        transformation_vectors = target_position - start_position
        normalized_transformation_vectors = transformation_vectors.clone()
        normalized_transformation_vectors.normalize_vectors()
        total_distance = transformation_vectors.longest_length()
        speed = self.__speed
        if speed_override:
            speed = speed_override
        elif distance_speed_multiplier:
            speed = total_distance * distance_speed_multiplier
        distance_traveled = 0
        while distance_traveled <= total_distance:
            distance_traveled += speed / self.__update_delay
            new_position = start_position + normalized_transformation_vectors * distance_traveled
            current_leg_height = get_height_for_step(distance_traveled, total_distance, leg_lift_height)
            for new_leg_pos, start_leg_pos in zip(new_position.get_legs_as_list(forward_legs), start_position.get_legs_as_list(forward_legs)):
                new_leg_pos.z = start_leg_pos.z + current_leg_height
            self.__last_written_position = new_position
            self.ik_driver.move_legs_synced(self.__last_written_position)
            # try:
                # pass
            # except (ValueError, ArithmeticError):
                # rospy.logerr("Ik failed")
            sleep(self.__update_delay * 0.001)

    def __go_to_relaxed(self, forward_legs, target_stance, speed_override=None, distance_speed_multiplier=None, leg_lift_height=2):
        start_position = self.__last_written_position.clone()
        target_position = start_position.update_from_other(target_stance, forward_legs)
        transformation_vectors = target_position - start_position
        normalized_transformation_vectors = transformation_vectors.clone()
        normalized_transformation_vectors.normalize_vectors()
        total_distance = transformation_vectors.longest_length()
        speed = self.__speed
        if speed_override:
            speed = speed_override
        elif distance_speed_multiplier:
            speed = total_distance * distance_speed_multiplier
        distance_traveled = 0
        while distance_traveled <= total_distance:
            distance_traveled += speed / self.__update_delay
            new_position = start_position + normalized_transformation_vectors * distance_traveled
            current_leg_height = get_height_for_step(distance_traveled, total_distance, leg_lift_height)
            for new_leg_pos, start_leg_pos in zip(new_position.get_legs_as_list(forward_legs),
                                                  start_position.get_legs_as_list(forward_legs)):
                new_leg_pos.z = start_leg_pos.z + current_leg_height
            self.__last_written_position = new_position
            self.ik_driver.move_legs_synced(self.__last_written_position)
            sleep(self.__update_delay * 0.001)

    def update_body_orientation(self, transform, rotation):
        self.__current_relaxed_position = RELAXED_POSITION.clone() \
                .transform(transform * -1) \
                .rotate(rotation)
        self.__execute_move(self.__current_relaxed_position)

    def __execute_move(self, target_position, speed_override=None):
        speed = self.__speed
        if speed_override:
            speed = speed_override
        while self.__last_written_position.move_towards(target_position, speed * 0.001 * self.__update_delay):
            self.ik_driver.move_legs_synced(self.__last_written_position)
            sleep(self.__update_delay * 0.001)
        self.ik_driver.move_legs_synced(self.__last_written_position)
        sleep(self.__update_delay * 0.001)

    def __get_next_leg_combo(self):
        self.__last_used_forward_legs = LegFlags.RIGHT_TRIPOD if self.__last_used_forward_legs == LegFlags.LEFT_TRIPOD else LegFlags.LEFT_TRIPOD
        return self.__last_used_forward_legs
