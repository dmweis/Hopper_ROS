import rospy
from std_msgs.msg import String


class SoundPlayer(object):
    def __init__(self):
        super(SoundPlayer, self).__init__()
        self.publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)

    def say(self, file_name):
        self.publisher.publish(file_name)
