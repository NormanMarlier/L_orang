# This class implement a compatible Python2-Python3 ROS node to send
import rospy
from std_msgs.msg import Float32MultiArray
import logging
logging.basicConfig(level=logging.INFO)


class RobotController(object):

    def __init__(self):
        """

        """
        # Publisher
        self.pub = rospy.Publisher('controller-lorang', Float32MultiArray, queue_size=10)

        # Cmd motors
        self.cmd_motors = Float32MultiArray()

        logging.info("The robot controller is initialized")

    def move_to(self, pose):
        """

        :param pose: A three value array
        :return:
        """
        self.cmd_motors.data = pose
        logging.info("Cmd motors : %s", self.cmd_motors.data)
        self.pub.publish(self.cmd_motors)