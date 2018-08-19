import rospy
from threading import Thread


class MessagePublisher(Thread):
    def __init__(self):
        super(MessagePublisher, self).__init__()
        self.keep_running = True
        self.publishers = []
        self.publish_rate = rospy.Rate(30)
        self.start()

    def register_publisher(self, publisher):
        self.publishers.append(publisher)

    def run(self):
        while not rospy.is_shutdown() and self.keep_running:
            for publisher in self.publishers:
                try:
                    publisher.publish()
                except rospy.ROSException as e:
                    rospy.logdebug("Hexapod controller failed to send data: " + str(e))
            self.publish_rate.sleep()

    def stop(self):
        self.keep_running = False
