import math
import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "left_front_coxa_joint",
    "left_front_femur_joint",
    "left_front_tibia_joint",
    "right_front_coxa_joint",
    "right_front_femur_joint",
    "right_front_tibia_joint",
    "left_middle_coxa_joint",
    "left_middle_femur_joint",
    "left_middle_tibia_joint",
    "right_middle_coxa_joint",
    "right_middle_femur_joint",
    "right_middle_tibia_joint",
    "left_rear_coxa_joint",
    "left_rear_femur_joint",
    "left_rear_tibia_joint",
    "right_rear_coxa_joint",
    "right_rear_femur_joint",
    "right_rear_tibia_joint",
]


class JointStatePublisher(object):
    def __init__(self, message_publisher):
        super(JointStatePublisher, self).__init__()
        self._publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._last_message = JointState()
        self.tibia_offset = rospy.get_param("tibia_offset")
        self.femur_offset = rospy.get_param("femur_offset")
        message_publisher.register_publisher(self)

    def update_joint_states(self, motor_positions):
        """
        :param motor_positions: positions of motors
        :return:
        """
        joint_positions = [
            motor_positions.left_front.coxa - 150,
            motor_positions.left_front.femur - 150 + self.femur_offset,
            motor_positions.left_front.tibia - 150 + 45 - self.tibia_offset,
            motor_positions.right_front.coxa - 150,
            motor_positions.right_front.femur - 150 - self.femur_offset,
            motor_positions.right_front.tibia - 150 - 45 + self.tibia_offset,
            motor_positions.left_middle.coxa - 150,
            motor_positions.left_middle.femur - 150 + self.femur_offset,
            motor_positions.left_middle.tibia - 150 + 45 - self.tibia_offset,

            motor_positions.right_middle.coxa - 150,
            motor_positions.right_middle.femur - 150 - self.femur_offset,
            motor_positions.right_middle.tibia - 150 - 45 + self.tibia_offset,
            motor_positions.left_rear.coxa - 150,
            motor_positions.left_rear.femur - 150 + self.femur_offset,
            motor_positions.left_rear.tibia - 150 + 45 - self.tibia_offset,
            motor_positions.right_rear.coxa - 150,
            motor_positions.right_rear.femur - 150 - self.femur_offset,
            motor_positions.right_rear.tibia - 150 - 45 + self.tibia_offset
        ]
        joint_state = JointState()
        joint_state.name = JOINT_NAMES
        joint_state.position = map(math.radians, joint_positions)
        joint_state.velocity = []
        joint_state.effort = []
        self._last_message = joint_state

    def publish(self):
        self._last_message.header.stamp = rospy.Time.now()
        self._publisher.publish(self._last_message)
