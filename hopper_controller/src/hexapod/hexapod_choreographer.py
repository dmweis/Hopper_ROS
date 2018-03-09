from __future__ import division
from __future__ import absolute_import
from .hexapod_ik_driver import  Vector3, Vector2, LegFlags
import rospy


def execute_choreography(gait_engine, choreography_name):
    happy_dance(gait_engine)


def happy_dance(gait_engine):
    gait_engine.reset_body_pose()
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

    gait_engine.move_to_new_pose(right_up_pose, 22)
    for i in range(4):
        gait_engine.move_to_new_pose(left_up_pose, 22)
        gait_engine.move_to_new_pose(right_up_pose, 22)
    gait_engine.reset_body_pose()

