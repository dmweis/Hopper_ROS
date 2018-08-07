import rospy
from threading import Thread


class MessagePublisher(Thread):
    def __init__(self):
        super(MessagePublisher, self).__init__()
        self.publishers = []
        self.publish_rate = rospy.Rate(30)
        self.start()

    def register_publisher(self, publisher):
        self.publishers.append(publisher)

    def run(self):
        while not rospy.is_shutdown():
            for publisher in self.publishers:
                try:
                    publisher.publish()
                except rospy.ROSException as e:
                    rospy.logdebug("Hexapod controller failed to send data: " + str(e))
            self.publish_rate.sleep()


DEFAULT_MESSAGE_PUBLISHER = None


def get_default_message_publisher():
    global DEFAULT_MESSAGE_PUBLISHER
    if DEFAULT_MESSAGE_PUBLISHER is None:
        DEFAULT_MESSAGE_PUBLISHER = MessagePublisher()
    return DEFAULT_MESSAGE_PUBLISHER
