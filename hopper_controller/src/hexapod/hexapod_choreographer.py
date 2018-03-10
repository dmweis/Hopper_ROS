from __future__ import division
from __future__ import absolute_import
from .hexapod_ik_driver import  Vector3, Vector2, LegFlags
import rospy


def happy_hand_dance(gait_engine):
    speed = 22
    gait_engine.reset_body_pose(speed)
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
    gait_engine.reset_body_pose(speed)


def happy_dance(gait_engine):
    speed = 18
    gait_engine.reset_body_pose(speed)
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
    gait_engine.reset_body_pose(speed)


def happy_spin(gait_engine):
    speed = 18
    gait_engine.reset_body_pose(speed)
    turned_left = gait_engine.get_relaxed_pose().rotate(Vector3(z=3))
    turned_right = gait_engine.get_relaxed_pose().rotate(Vector3(z=-3))
    for i in range(4):
        gait_engine.move_to_new_pose(turned_left, speed)
        gait_engine.move_to_new_pose(turned_right, speed)
    gait_engine.reset_body_pose(speed)


def sad_emote(gait_engine):
    speed = 3
    gait_engine.reset_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose()
    gait_engine.move_to_new_pose(relaxed_pose.rotate(Vector3(y=8)), speed)
    gait_engine.move_to_new_pose(relaxed_pose.transform(Vector3(z=3)), speed)
    gait_engine.reset_body_pose(speed)


def wave_hi(gait_engine):
    speed = 10
    gait_engine.reset_body_pose()
    lifted_pose = gait_engine.get_relaxed_pose() \
        .rotate(Vector3(y=-5)) \
        .rotate(Vector3(x=-5)) \
        .transform(Vector3(z=-2)) \
        .transform(Vector3(4, -4), LegFlags.RIGHT_FRONT)
    gait_engine.move_to_new_pose(lifted_pose, speed)
    lifted_pose.right_front.z = -2
    gait_engine.move_to_new_pose(lifted_pose, 18)

    paw_lifted = lifted_pose.clone()
    paw_lifted.right_front.z = 0

    paw_lowered = lifted_pose.clone()
    paw_lowered.right_front.z = -4
    for i in range(4):
        gait_engine.move_to_new_pose(paw_lifted, speed)
        gait_engine.move_to_new_pose(paw_lowered, speed)
    gait_engine.reset_body_pose()


moves = {
    "happy_hand_dance": happy_hand_dance,
    "happy_dance": happy_dance,
    "happy_spin": happy_spin,
    "sad_emote": sad_emote,
    "wave_hi": wave_hi
}


def execute_choreography(gait_engine, choreography_name):
    try:
        moves[choreography_name](gait_engine)
    except KeyError:
        rospy.logerr("Key: (%s) not present in moves dictionary", choreography_name)
