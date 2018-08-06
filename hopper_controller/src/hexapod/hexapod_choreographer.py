from __future__ import division
from __future__ import absolute_import
from .hexapod_ik_driver import Vector3, Vector2, LegFlags
import rospy


def happy_hand_dance(gait_engine):
    speed = 22
    gait_engine.reset_relaxed_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose()
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
        gait_engine.move_to_new_pose(hands_up, speed)
        gait_engine.move_to_new_pose(hands_down, speed)
    gait_engine.reset_relaxed_body_pose(speed)


def happy_dance(gait_engine):
    speed = 18
    gait_engine.reset_relaxed_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose()
    legs_separated_pose = relaxed_pose.clone() \
        .rotate(Vector3(y=-8))

    lean_left = legs_separated_pose.clone() \
        .rotate(Vector3(x=-2))

    lean_right = legs_separated_pose.clone() \
        .rotate(Vector3(x=2))

    for i in range(4):
        gait_engine.move_to_new_pose(lean_right, speed)
        gait_engine.move_to_new_pose(lean_left, speed)
    gait_engine.reset_relaxed_body_pose(speed)


def happy_spin(gait_engine):
    speed = 18
    gait_engine.reset_relaxed_body_pose(speed)
    turned_left = gait_engine.get_relaxed_pose().rotate(Vector3(z=3))
    turned_right = gait_engine.get_relaxed_pose().rotate(Vector3(z=-3))
    for i in range(4):
        gait_engine.move_to_new_pose(turned_left, speed)
        gait_engine.move_to_new_pose(turned_right, speed)
    gait_engine.reset_relaxed_body_pose(speed)


def sad_emote(gait_engine):
    speed = 3
    gait_engine.reset_relaxed_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose()
    gait_engine.move_to_new_pose(relaxed_pose.rotate(Vector3(y=8)), speed)
    gait_engine.move_to_new_pose(relaxed_pose.transform(Vector3(z=3)), speed)
    gait_engine.reset_relaxed_body_pose(speed)


def wave_hi(gait_engine):
    speed = 12
    gait_engine.reset_relaxed_body_pose()
    lifted_pose = gait_engine.get_relaxed_pose() \
        .rotate(Vector3(y=-5)) \
        .rotate(Vector3(x=-5)) \
        .transform(Vector3(z=-2)) \
        .transform(Vector3(6, -6), LegFlags.RIGHT_FRONT)
    lifted_pose.right_front.z = -4
    gait_engine.move_to_new_pose(lifted_pose, speed)

    paw_lifted = lifted_pose.clone()
    paw_lifted.right_front.z = 0

    paw_lowered = lifted_pose.clone()
    paw_lowered.right_front.z = -4
    for i in range(3):
        gait_engine.move_to_new_pose(paw_lifted, speed)
        gait_engine.move_to_new_pose(paw_lowered, speed)
    gait_engine.reset_relaxed_body_pose()


def lift_legs(gait_engine):
    speed = 12
    gait_engine.reset_relaxed_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose().clone()
    for leg in LegFlags.get_legs_as_list(LegFlags.ALL):
        lifted_leg = relaxed_pose.transform(Vector3(z=5), leg).clone()
        gait_engine.move_to_new_pose(lifted_leg, speed)
        gait_engine.move_to_new_pose(relaxed_pose, speed)
    gait_engine.reset_relaxed_body_pose()


def roar(gait_engine):
    slow_speed = 12
    fast_speed = 20
    gait_engine.reset_relaxed_body_pose(slow_speed)
    normal_pose = gait_engine.get_relaxed_pose()
    lifted_middle = normal_pose \
        .transform(Vector3(x=4, z=4), LegFlags.MIDDLE)
    grounded_middle_front = normal_pose \
        .transform(Vector3(x=4), LegFlags.MIDDLE)
    lifted_front = grounded_middle_front \
        .transform(Vector3(z=8, x=8), LegFlags.FRONT) \
        .rotate(Vector3(y=-14)) \
        .transform(Vector3(y=-6), LegFlags.LEFT_FRONT) \
        .transform(Vector3(y=6), LegFlags.RIGHT_FRONT)
    lifted_left = lifted_front \
        .transform(Vector3(z=-2), LegFlags.LEFT_FRONT) \
        .transform(Vector3(z=2), LegFlags.RIGHT_FRONT)
    lifted_right = lifted_front \
        .transform(Vector3(z=2), LegFlags.LEFT_FRONT) \
        .transform(Vector3(z=-2), LegFlags.RIGHT_FRONT)
    gait_engine.move_to_new_pose(lifted_middle, slow_speed)
    gait_engine.move_to_new_pose(grounded_middle_front, slow_speed)
    gait_engine.move_to_new_pose(lifted_front, fast_speed)
    for i in range(6):
        gait_engine.move_to_new_pose(lifted_left, fast_speed)
        gait_engine.move_to_new_pose(lifted_right, fast_speed)
    gait_engine.move_to_new_pose(lifted_front, slow_speed)
    gait_engine.move_to_new_pose(grounded_middle_front, slow_speed)
    gait_engine.move_to_new_pose(lifted_middle, slow_speed)
    gait_engine.reset_relaxed_body_pose()


def hump(gait_engine):
    speed = 15
    gait_engine.reset_relaxed_body_pose(speed)
    normal_pose = gait_engine.get_relaxed_pose()
    forward_hump = normal_pose \
        .transform(Vector3(x=4)) \
        .rotate(Vector3(x=6))
    backwards_hump = normal_pose \
        .transform(Vector3(x=-4)) \
        .rotate(Vector3(x=-6))
    for i in range(6):
        gait_engine.move_to_new_pose(forward_hump, speed)
        gait_engine.move_to_new_pose(backwards_hump, speed)
    gait_engine.reset_relaxed_body_pose()


moves = {
    "happy_hand_dance": happy_hand_dance,
    "happy_dance": happy_dance,
    "happy_spin": happy_spin,
    "sad_emote": sad_emote,
    "wave_hi": wave_hi,
    "lifted_legs": lift_legs,
    "hump": hump,
    "roar": roar
}


def execute_choreography(gait_engine, choreography_name):
    try:
        moves[choreography_name](gait_engine)
    except KeyError:
        rospy.logerr("Key: (%s) not present in moves dictionary", choreography_name)
