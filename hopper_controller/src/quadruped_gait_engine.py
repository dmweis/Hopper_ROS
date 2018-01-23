from __future__ import division
from __future__ import absolute_import
from quadruped_ik_driver import LegPositions, IkDriver, Vector3, Vector2, LegFlags
import threading
from time import sleep

INTERPOLATION_FREQUENCY = 30

LEG_HEIGHT = -9
LEG_DISTANCE_LONGITUDAL = 15
LEG_DISTANCE_LATERAL = 15

RELAXED_POSITION = LegPositions(
    Vector3(LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
    Vector3(-LEG_DISTANCE_LATERAL, -LEG_DISTANCE_LONGITUDAL, LEG_HEIGHT),
)

class GaitController(threading.Thread):
    def __init__(self, ik_driver):
        super(GaitController, self).__init__()
        self.ik_driver = ik_driver
        self.direction = Vector2(0, 0)
        self.rotation = 0
        self.__last_written_position = RELAXED_POSITION.clone()
        self.__last_used_forward_legs = LegFlags.RF_LR_CROSS
        self.__speed = 60
        self.__update_delay = 1000 / 30
        self.__keep_running = True
        self.ik_driver.setup()
        self.start()

    def run(self):
        while self.__keep_running:
            if not self.direction.is_zero() or self.rotation != 0:
                self.__execute_step(self.direction, self.rotation, self.__get_next_leg_combo())
            else:
                sleep(self.__update_delay * 0.001)

    def stop(self):
        self.__keep_running = False
        self.join()
        self.ik_driver.disable_motors()
        self.ik_driver.close()

    def __execute_step(self, direction, angle, forward_legs, leg_lift_height = 2):
        leg_shift = 0.25
        angle = angle / 4.0
        backwards_moving_legs = LegFlags.RF_LR_CROSS if forward_legs == LegFlags.LF_RR_CROSS else LegFlags.LF_RR_CROSS
        # lift
        next_step = RELAXED_POSITION.clone()\
            .transform(Vector3(leg_shift * direction.x, leg_shift * direction.y, leg_lift_height), forward_legs) \
            .rotate(-angle, forward_legs) \
            .transform(Vector3(-leg_shift * direction.x, -leg_shift * direction.y, 0), backwards_moving_legs) \
            .rotate(angle, backwards_moving_legs)
        self.__execute_mode(next_step)
        # lower
        next_step = next_step.clone() \
            .transform(Vector3(leg_shift * direction.x, leg_shift * direction.y, -leg_lift_height), forward_legs) \
            .rotate(-angle, forward_legs) \
            .transform(Vector3(-leg_shift * direction.x, -leg_shift * direction.y, 0), backwards_moving_legs) \
            .rotate(angle, backwards_moving_legs)
        self.__execute_mode(next_step)
        # Move body
        next_step = next_step.clone() \
            .transform(Vector3(-leg_shift * direction.x * 2, -leg_shift * direction.y * 2, 0))
        self.__execute_mode(next_step)

    def __execute_mode(self, target_position):
        while self.__last_written_position.move_towards(target_position, self.__speed * 0.001 * self.__update_delay):
            # print(self.__last_written_position)
            # print("target: " + str(target_position))
            self.ik_driver.move_legs_synced(self.__last_written_position)
            sleep(self.__update_delay * 0.001)
        # print(self.__last_written_position)
        self.ik_driver.move_legs_synced(self.__last_written_position)
        sleep(self.__update_delay * 0.001)
        # print("done ------------------------------------------------")

    def __get_next_leg_combo(self):
        self.__last_used_forward_legs = LegFlags.RF_LR_CROSS if self.__last_used_forward_legs == LegFlags.LF_RR_CROSS else LegFlags.LF_RR_CROSS
        return self.__last_used_forward_legs