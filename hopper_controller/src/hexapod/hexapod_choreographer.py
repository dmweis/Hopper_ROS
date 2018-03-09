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
    left_up_pose = legs_separated_pose.clone()
    left_up_pose.left_middle.z = 2
    left_up_pose.right_middle.z = -2

    right_up_pose = legs_separated_pose.clone()
    right_up_pose.left_middle.z = -2
    right_up_pose.right_middle.z = 2

    gait_engine.move_to_new_pose(right_up_pose, speed)
    for i in range(4):
        gait_engine.move_to_new_pose(left_up_pose, speed)
        gait_engine.move_to_new_pose(right_up_pose, speed)
    gait_engine.reset_body_pose(speed)


def happy_dance(gait_engine):
    speed = 12
    gait_engine.reset_body_pose(speed)
    relaxed_pose = gait_engine.get_relaxed_pose()
    legs_separated_pose = relaxed_pose.clone() \
        .transform(Vector3(y=-10), LegFlags.RIGHT_MIDDLE) \
        .transform(Vector3(y=10), LegFlags.LEFT_MIDDLE)
    legs_separated_pose.left_middle.z = 0
    legs_separated_pose.right_middle.z = 0

    down_pose = legs_separated_pose.clone() \
        .transform(Vector3(z=-3), LegFlags.FRONT | LegFlags.REAR) \
        .transform(Vector3(z=3), LegFlags.MIDDLE)

    up_pose = legs_separated_pose.clone() \
        .transform(Vector3(z=3), LegFlags.FRONT | LegFlags.REAR) \
        .transform(Vector3(z=-3), LegFlags.MIDDLE)

    gait_engine.move_to_new_pose(down_pose, speed)
    for i in range(4):
        gait_engine.move_to_new_pose(up_pose, speed)
        gait_engine.move_to_new_pose(down_pose, speed)
    gait_engine.reset_body_pose(speed)


moves = {
    "happy_hand_dance": happy_hand_dance,
    "happy_dance": happy_dance,
}


def execute_choreography(gait_engine, choreography_name):
    try:
        moves[choreography_name](gait_engine)
    except KeyError:
        rospy.logerr("Key: (%s) not present in moves dictionary", choreography_name)
