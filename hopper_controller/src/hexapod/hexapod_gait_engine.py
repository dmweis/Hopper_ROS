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
MIDDLE_LEG_LONGITUDAL_OFFSET = 7
LEG_DISTANCE_LATERAL = 18

RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
)

OFFSET_DISTANCE = 3
GROUND_LEG_HEIGHT = -3

GROUND_LEVEL_RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET + OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET - OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, GROUND_LEG_HEIGHT),
)

WIDER_RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL + OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(0, LEG_DISTANCE_LONGITUDAL + MIDDLE_LEG_LONGITUDAL_OFFSET + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(0, -LEG_DISTANCE_LONGITUDAL - MIDDLE_LEG_LONGITUDAL_OFFSET - OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, LEG_DISTANCE_LONGITUDAL + OFFSET_DISTANCE, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL - OFFSET_DISTANCE, -LEG_DISTANCE_LONGITUDAL - OFFSET_DISTANCE, LEG_HEIGHT),
)


def get_height_for_step(distance, full_step_length, height):
    distance = distance / full_step_length
    return math.sin(distance * math.pi) * height


class GaitController(threading.Thread):
    def __init__(self, gait_engine):
        super(GaitController, self).__init__()
        self.ready = False
        self._gait_engine = gait_engine
        self._keep_running = True
        self._relaxed = True
        self._direction = Vector2(0, 0)
        self._rotation = 0
        self._relaxed_transformation = Vector3(0, 0, 0)
        self._relaxed_rotation = Vector3(0, 0, 0)
        self._pose_update_ready = False
        self._telemetric_subscribers = []
        self._last_telemetrics_update_time = time()
        self._ros_timer = rospy.Rate(INTERPOLATION_FREQUENCY)
        self.start()

    def run(self):
        try:
            rospy.loginfo("Hexapod gait engine started")
            self._gait_engine.stand_up()
            while self._keep_running:
                if self._should_move():
                    self._gait_engine.step(self._direction, self._rotation)
                    self._relaxed = False
                elif not self._relaxed:
                    self._gait_engine.relax_next_leg()
                    if not self._should_move():
                        self._gait_engine.relax_next_leg()
                        self._relaxed = True
                elif self._pose_update_ready:
                    self._pose_update_ready = False
                    self._gait_engine.update_body_pose(self._relaxed_transformation, self._relaxed_rotation)
                else:
                    self._ros_timer.sleep()
                    if time() - self._last_telemetrics_update_time > 1:
                        self._last_telemetrics_update_time = time()
                        telemetrics = self._gait_engine.read_telemetrics()
                        for sub in self._telemetric_subscribers:
                            sub(telemetrics)
            self._gait_engine.sit_down()
        except Exception as e:
            self._log_current_state()
            rospy.logfatal("Gait engine loop failed " + str(e))
            rospy.signal_shutdown("Gait engine loop failed " + str(e))

    def set_relaxed_pose(self, transform, rotation):
        self._relaxed_transformation = transform
        self._relaxed_rotation = rotation
        self._pose_update_ready = True

    def set_direction(self, direction, rotation):
        self._direction = direction
        self._rotation = rotation

    def stop_moving(self):
        self._direction = Vector2(0, 0)
        self._rotation = 0

    def subscribe_to_telemetrics(self, callback):
        self._telemetric_subscribers.append(callback)

    def stop(self):
        self._keep_running = False
        self.join()
        self._gait_engine.stop()

    def _should_move(self):
        return not self._direction.is_zero() or self._rotation != 0

    def _log_current_state(self):
        data = {'direction': self._direction,
                'rotation': self._rotation,
                'relaxed_transformation': self._relaxed_transformation,
                'relaxed_rotation': self._relaxed_rotation}
        rospy.loginfo("Current hexapod state: %s", str(data))

class GaitEngine(object):
    def __init__(self, ik_driver):
        super(GaitEngine, self).__init__()
        self._ik_driver = ik_driver
        self._current_relaxed_position = RELAXED_POSITION.clone()
        self._last_used_forward_legs = LegFlags.LEFT_TRIPOD
        self._speed = 9
        self._update_delay = 1000 / INTERPOLATION_FREQUENCY
        self._ik_driver.setup()
        self._last_written_position = self._ik_driver.read_current_leg_positions()

    def stand_up(self):
        rospy.loginfo("Hexapod gait engine started")
        for leg in LegFlags.get_legs_as_list(LegFlags.ALL):
            new_position = self._last_written_position.clone().update_from_other(GROUND_LEVEL_RELAXED_POSITION, leg)
            self._execute_move(new_position, 6)
        self._execute_move(WIDER_RELAXED_POSITION.clone(), 6)
        self._go_to_relaxed(self._get_next_leg_combo(), self._current_relaxed_position, distance_speed_multiplier=2)
        self._go_to_relaxed(self._get_next_leg_combo(), self._current_relaxed_position, distance_speed_multiplier=2)
        rospy.loginfo("Hexapod ready")

    def step(self, direction, rotation):
        if direction.is_zero() and abs(rotation) > 8:
            # just rotation
            self._execute_step(direction, rotation, self._get_next_leg_combo(), distance_speed_multiplier=6)
        elif self.direction.length() > 5.5:
            # fast walking
            self._execute_step(direction, rotation, self._get_next_leg_combo(), distance_speed_multiplier=5)
        else:
            # regular walking
            self._execute_step(direction, rotation, self._get_next_leg_combo(), distance_speed_multiplier=3)

    def relax_next_leg(self):
        self._go_to_relaxed(self._get_next_leg_combo(), self._current_relaxed_position, distance_speed_multiplier=2)

    def update_body_pose(self, transform, rotation):
        self._current_relaxed_position = RELAXED_POSITION.clone() \
            .transform(transform * -1) \
            .rotate(rotation)
        self._execute_move(self._current_relaxed_position)

    def read_telemetrics(self):
        return self._ik_driver.read_telemetrics()

    def sit_down(self):
        self.update_body_pose(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self._go_to_relaxed(self._get_next_leg_combo(), WIDER_RELAXED_POSITION, distance_speed_multiplier=2)
        self._go_to_relaxed(self._get_next_leg_combo(), WIDER_RELAXED_POSITION, distance_speed_multiplier=2)
        self._execute_move(GROUND_LEVEL_RELAXED_POSITION.clone(), 3)

    def stop(self, disable_motors=True):
        if disable_motors:
            self._ik_driver.disable_motors()
        self._ik_driver.close()

    def _execute_step(self, direction, angle, forward_legs, speed_override=None, distance_speed_multiplier=None, leg_lift_height=2):
        backwards_legs = LegFlags.RIGHT_TRIPOD if forward_legs == LegFlags.LEFT_TRIPOD else LegFlags.LEFT_TRIPOD
        start_position = self._last_written_position.clone()
        target_position = self._current_relaxed_position.clone() \
            .transform(Vector3(direction.x / 2, direction.y / 2, 0), forward_legs) \
            .turn(-angle / 2, forward_legs) \
            .transform(Vector3(-direction.x / 2, -direction.y / 2, 0), backwards_legs) \
            .turn(angle / 2, backwards_legs)
        transformation_vectors = target_position - start_position
        normalized_transformation_vectors = transformation_vectors.clone()
        normalized_transformation_vectors.normalize_vectors()
        total_distance = transformation_vectors.longest_length()
        speed = self._speed
        if speed_override:
            speed = speed_override
        elif distance_speed_multiplier:
            speed = total_distance * distance_speed_multiplier
        distance_traveled = 0
        while distance_traveled <= total_distance:
            distance_traveled += speed / self._update_delay
            new_position = start_position + normalized_transformation_vectors * distance_traveled
            current_leg_height = get_height_for_step(distance_traveled, total_distance, leg_lift_height)
            for new_leg_pos, start_leg_pos in zip(new_position.get_legs_as_list(forward_legs), start_position.get_legs_as_list(forward_legs)):
                new_leg_pos.z = start_leg_pos.z + current_leg_height
            self._last_written_position = new_position
            self._ik_driver.move_legs_synced(self._last_written_position)
            sleep(self._update_delay * 0.001)

    def _go_to_relaxed(self, forward_legs, target_stance, speed_override=None, distance_speed_multiplier=None, leg_lift_height=2):
        start_position = self._last_written_position.clone()
        target_position = start_position.update_from_other(target_stance, forward_legs)
        transformation_vectors = target_position - start_position
        normalized_transformation_vectors = transformation_vectors.clone()
        normalized_transformation_vectors.normalize_vectors()
        total_distance = transformation_vectors.longest_length()
        speed = self._speed
        if speed_override:
            speed = speed_override
        elif distance_speed_multiplier:
            speed = total_distance * distance_speed_multiplier
        distance_traveled = 0
        while distance_traveled <= total_distance:
            distance_traveled += speed / self._update_delay
            new_position = start_position + normalized_transformation_vectors * distance_traveled
            current_leg_height = get_height_for_step(distance_traveled, total_distance, leg_lift_height)
            for new_leg_pos, start_leg_pos in zip(new_position.get_legs_as_list(forward_legs),
                                                  start_position.get_legs_as_list(forward_legs)):
                new_leg_pos.z = start_leg_pos.z + current_leg_height
            self._last_written_position = new_position
            self._ik_driver.move_legs_synced(self._last_written_position)
            sleep(self._update_delay * 0.001)

    def _execute_move(self, target_position, speed_override=None):
        speed = self._speed
        if speed_override:
            speed = speed_override
        while self._last_written_position.move_towards(target_position, speed * 0.001 * self._update_delay):
            self._ik_driver.move_legs_synced(self._last_written_position)
            sleep(self._update_delay * 0.001)
        self._ik_driver.move_legs_synced(self._last_written_position)
        sleep(self._update_delay * 0.001)

    def _get_next_leg_combo(self):
        self._last_used_forward_legs = LegFlags.RIGHT_TRIPOD if self._last_used_forward_legs == LegFlags.LEFT_TRIPOD else LegFlags.LEFT_TRIPOD
        return self._last_used_forward_legs
