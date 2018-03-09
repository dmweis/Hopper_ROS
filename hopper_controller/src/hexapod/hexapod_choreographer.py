from __future__ import division
from __future__ import absolute_import
from .hexapod_ik_driver import  Vector3, Vector2, LegFlags
import rospy


def execute_choreography(gait_engine, choreography_name):
    happy_dance(gait_engine)


def happy_dance(gait_engine):
    gait_engine.reset_body_pose()
    rospy.sleep(1)
    gait_engine.update_body_pose(Vector3(z=-9), Vector3(), LegFlags.MIDDLE)
    rospy.sleep(1)
    for i in xrange(4):
        gait_engine.update_body_pose(Vector3(z=-7), Vector3(), LegFlags.LEFT_MIDDLE, execute_motion=False)
        gait_engine.update_body_pose(Vector3(z=-11), Vector3(), LegFlags.RIGHT_MIDDLE)
        rospy.sleep(1)
        gait_engine.update_body_pose(Vector3(z=-11), Vector3(), LegFlags.LEFT_MIDDLE, execute_motion=False)
        gait_engine.update_body_pose(Vector3(z=-7), Vector3(), LegFlags.RIGHT_MIDDLE)
        rospy.sleep(1)
    rospy.sleep(3)
    gait_engine.reset_body_pose()

