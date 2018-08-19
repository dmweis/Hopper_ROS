import rospy
from std_msgs.msg import String


class SoundPlayer(object):
    def __init__(self, sound_on):
        super(SoundPlayer, self).__init__()
        self.sound_on = sound_on
        self.publisher = rospy.Publisher('hopper_play_sound', String, queue_size=5)

    def say(self, file_name):
        if self.sound_on:
            self.publisher.publish(file_name)
