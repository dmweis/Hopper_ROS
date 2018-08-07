import rospy
from sensor_msgs.msg import JointState
from .telemetric_updater import get_default_message_publisher


class JointStatePublisher(object):
    def __init__(self):
        super(JointStatePublisher, self).__init__()
        self._publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._last_message = JointState()
        get_default_message_publisher().register_publisher(self)

    def update_joint_states(self, joint_names, joint_positions):
        """
        :param joint_names: list of names of the joints
        :param joint_positions: positions of joints
        :return:
        """
        joint_state = JointState()
        joint_state.name = joint_names
        joint_state.position = joint_positions
        joint_state.velocity = []
        joint_state.effort = []
        self._last_message = joint_state

    def publish(self):
        self._last_message.header.stamp = rospy.Time.now()
        self._publisher.publish(self._last_message)
