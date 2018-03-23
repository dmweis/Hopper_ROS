#!/usr/bin/env python

import rospy
from hopper_emotion_core.msg import EmotionChange
from std_msgs.msg import String


class EmotionalReactionController(object):
    def __init__(self):
        rospy.init_node("Emotional_reaction_controller")
        self.emotion_change_subscriber = rospy.Subscriber("emotional_change", EmotionChange, self.new_emotion_change, queue_size=10)
        self.dance_command_publisher = rospy.Publisher("hopper_schedule_move", String, queue_size=10)
        rospy.spin()

    def new_emotion_change(self, emotion_change):
        if emotion_change.person_name == "David":
            if emotion_change.previous_emotion != "happiness" and emotion_change.new_emotion == "happiness":
                self.dance_command_publisher.publish("happy_dance")
            if emotion_change.previous_emotion != "surprise" and emotion_change.new_emotion == "surprise":
                self.dance_command_publisher.publish("happy_hand_dance")
            if emotion_change.previous_emotion != "sadness" and emotion_change.new_emotion == "sadness":
                self.dance_command_publisher.publish("sad_emote")

if __name__ == "__main__":
    EmotionalReactionController()