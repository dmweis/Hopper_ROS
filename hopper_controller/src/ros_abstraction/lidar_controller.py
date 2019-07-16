import rospy
from std_srvs.srv import Empty


class LidarController(object):
    def __init__(self):
        super(LidarController, self).__init__()
        self.lidar_on = rospy.ServiceProxy("start_motor", Empty)
        self.lidar_off = rospy.ServiceProxy("stop_motor", Empty)

    def start(self):
        self.lidar_on()

    def stop(self):
        self.lidar_off()
