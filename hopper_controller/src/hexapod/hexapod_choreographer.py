from __future__ import division
from __future__ import absolute_import
import random
from .hexapod_ik_driver import Vector3, LegFlags
from std_msgs.msg import String
import rospy


class Choreographer(object):
    def __init__(self, gait_engine):
        super(Choreographer, self).__init__()
        self.gait_engine = gait_engine
        self.cancelation_sentinel = False
        self.speak_publisher = rospy.Publisher("hopper_play_sound", String, queue_size=1)
        self.dance_lookup = {
            "happy_hand_dance": self.happy_hand_dance,
            "happy_dance": self.happy_dance,
            "happy_spin": self.happy_spin,
            "sad_emote": self.sad_emote,
            "wave_hi": self.wave_hi,
            "lifted_legs": self.lift_legs,
            "hump": self.hump,
            "roar": self.roar
        }

    def execute_choreography(self, choreography_name):
        original_pose = self.gait_engine.get_relaxed_pose()
        if choreography_name.lower() == "random":
            choreography_name = random.choice(self.dance_lookup.keys())
        try:
            self.dance_lookup[choreography_name]()
        except KeyError:
            rospy.logerr("Key: (%s) not present in moves dictionary", choreography_name)
        except RuntimeWarning:
            rospy.logerr("Operation interrupted")
            self.gait_engine.move_to_new_pose(original_pose, 15)

    def cancel_move(self):
        self.cancelation_sentinel = True

    def check_cancel(self):
        if self.cancelation_sentinel:
            self.cancelation_sentinel = False
            raise RuntimeWarning("Operation canceled")

    def happy_hand_dance(self):
        speed = 22
        relaxed_pose = self.gait_engine.get_relaxed_pose()
        legs_separated_pose = relaxed_pose.clone() \
            .transform(Vector3(y=-10), LegFlags.RIGHT_MIDDLE) \
            .transform(Vector3(y=10), LegFlags.LEFT_MIDDLE)
        hands_up = legs_separated_pose.clone()
        hands_up.left_middle.z = -2
        hands_up.right_middle.z = -2

        hands_down = legs_separated_pose.clone()
        hands_down.left_middle.z = 2
        hands_down.right_middle.z = 2

        for i in range(4):
            self.gait_engine.move_to_new_pose(hands_up, speed)
            self.gait_engine.move_to_new_pose(hands_down, speed)
            self.check_cancel()
        self.gait_engine.move_to_new_pose(relaxed_pose, speed)

    def happy_dance(self):
        speed = 18
        relaxed_pose = self.gait_engine.get_relaxed_pose()
        legs_separated_pose = relaxed_pose.clone() \
            .rotate(Vector3(y=-8))

        lean_left = legs_separated_pose.clone() \
            .rotate(Vector3(x=-2))

        lean_right = legs_separated_pose.clone() \
            .rotate(Vector3(x=2))

        for i in range(4):
            self.gait_engine.move_to_new_pose(lean_right, speed)
            self.gait_engine.move_to_new_pose(lean_left, speed)
            self.check_cancel()
        self.gait_engine.move_to_new_pose(relaxed_pose, speed)

    def happy_spin(self):
        speed = 18
        relaxed_pose = self.gait_engine.get_relaxed_pose()
        turned_left = self.gait_engine.get_relaxed_pose().rotate(Vector3(z=3))
        turned_right = self.gait_engine.get_relaxed_pose().rotate(Vector3(z=-3))
        for i in range(4):
            self.gait_engine.move_to_new_pose(turned_left, speed)
            self.gait_engine.move_to_new_pose(turned_right, speed)
        self.gait_engine.move_to_new_pose(relaxed_pose, speed)

    def sad_emote(self):
        speed = 3
        original_pose = self.gait_engine.get_relaxed_pose()
        self.gait_engine.reset_relaxed_body_pose(speed)
        relaxed_pose = self.gait_engine.get_relaxed_pose()
        self.gait_engine.move_to_new_pose(relaxed_pose.rotate(Vector3(y=8)), speed)
        self.check_cancel()
        self.gait_engine.move_to_new_pose(relaxed_pose.transform(Vector3(z=3)), speed)
        self.check_cancel()
        self.gait_engine.move_to_new_pose(original_pose, speed)

    def wave_hi(self):
        speed = 12
        original_pose = self.gait_engine.get_relaxed_pose()
        lifted_pose = self.gait_engine.get_relaxed_pose() \
            .rotate(Vector3(y=-5)) \
            .rotate(Vector3(x=-5)) \
            .transform(Vector3(z=-2)) \
            .transform(Vector3(10, -2), LegFlags.RIGHT_FRONT)
        lifted_pose.right_front.z = 4
        self.gait_engine.move_to_new_pose(lifted_pose, speed)
        self.check_cancel()

        paw_lifted = lifted_pose.clone()
        paw_lifted.right_front.y -= 2

        paw_lowered = lifted_pose.clone()
        paw_lowered.right_front.z = 0
        wave_speed = 25
        for i in range(2):
            self.gait_engine.move_to_new_pose(paw_lifted, wave_speed)
            self.gait_engine.move_to_new_pose(paw_lowered, wave_speed)
            self.check_cancel()
        self.speak_publisher.publish("Turret_turret_active_1")
        for i in range(4):
            self.gait_engine.move_to_new_pose(paw_lifted, wave_speed)
            self.gait_engine.move_to_new_pose(paw_lowered, wave_speed)
            self.check_cancel()
        self.gait_engine.move_to_new_pose(original_pose, speed)

    def lift_legs(self):
        speed = 12
        self.gait_engine.reset_relaxed_body_pose(speed)
        relaxed_pose = self.gait_engine.get_relaxed_pose().clone()
        for leg in LegFlags.get_legs_as_list(LegFlags.ALL):
            lifted_leg = relaxed_pose.transform(Vector3(z=5), leg).clone()
            self.gait_engine.move_to_new_pose(lifted_leg, speed)
            self.gait_engine.move_to_new_pose(relaxed_pose, speed)
            self.check_cancel()
        self.gait_engine.reset_relaxed_body_pose()

    def roar(self):
        slow_speed = 12
        fast_speed = 25
        normal_pose = self.gait_engine.get_relaxed_pose()
        lifted_middle = normal_pose \
            .transform(Vector3(x=4, z=4), LegFlags.MIDDLE)
        grounded_middle_front = normal_pose \
            .transform(Vector3(x=4), LegFlags.MIDDLE)
        lifted_front = grounded_middle_front \
            .transform(Vector3(z=12, x=12), LegFlags.FRONT) \
            .rotate(Vector3(y=-14)) \
            .transform(Vector3(y=-6), LegFlags.LEFT_FRONT) \
            .transform(Vector3(y=6), LegFlags.RIGHT_FRONT)
        lifted_left = lifted_front \
            .transform(Vector3(z=-2), LegFlags.LEFT_FRONT) \
            .transform(Vector3(z=2), LegFlags.RIGHT_FRONT)
        lifted_right = lifted_front \
            .transform(Vector3(z=2), LegFlags.LEFT_FRONT) \
            .transform(Vector3(z=-2), LegFlags.RIGHT_FRONT)
        self.gait_engine.move_to_new_pose(lifted_middle, slow_speed)
        self.gait_engine.move_to_new_pose(grounded_middle_front, slow_speed)
        self.check_cancel()
        self.gait_engine.move_to_new_pose(lifted_front, fast_speed)
        for i in range(6):
            self.gait_engine.move_to_new_pose(lifted_left, fast_speed)
            self.gait_engine.move_to_new_pose(lifted_right, fast_speed)
            self.check_cancel()
        self.gait_engine.move_to_new_pose(lifted_front, slow_speed)
        self.gait_engine.move_to_new_pose(grounded_middle_front, slow_speed)
        self.gait_engine.move_to_new_pose(lifted_middle, slow_speed)
        self.gait_engine.move_to_new_pose(normal_pose, slow_speed)

    def hump(self):
        speed = 13
        hacklab_speaker = rospy.Publisher("hacklab/play", String, queue_size=1)
        normal_pose = self.gait_engine.get_relaxed_pose()
        forward_hump = normal_pose \
            .transform(Vector3(x=-3)) \
            .rotate(Vector3(y=-10))
        backwards_hump = normal_pose \
            .transform(Vector3(x=3))
        for i in range(10):
            self.gait_engine.move_to_new_pose(forward_hump, speed)
            speed = speed + 1
            self.check_cancel()
            self.gait_engine.move_to_new_pose(backwards_hump, speed)
            speed = speed + 1
            self.check_cancel()
        self.speak_publisher.publish("Turret_turret_active_3")
        hacklab_speaker.publish("Turret_turret_active_3.wav")
        self.gait_engine.move_to_new_pose(forward_hump, speed)
        for i in range(5):
            self.gait_engine.move_to_new_pose(forward_hump.transform(Vector3(z=-1)), speed)
            self.gait_engine.move_to_new_pose(forward_hump.transform(Vector3(z=1)), speed)
            self.check_cancel()
        self.gait_engine.move_to_new_pose(backwards_hump, 9)
        self.gait_engine.move_to_new_pose(normal_pose, speed)
