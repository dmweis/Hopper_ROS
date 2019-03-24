import rospy

from std_msgs.msg import Bool


class ControllerTelemetryPublisher(object):
    def __init__(self):
        super(ControllerTelemetryPublisher, self).__init__()
        self.ready_publisher = rospy.Publisher("hopper/main_controller_ready", Bool, latch=True, queue_size=2)

    def ready(self, state=True):
        self.ready_publisher.publish(Bool(state))
