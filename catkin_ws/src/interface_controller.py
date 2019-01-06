#!/usr/bin/env python

"""

This script allows the user to send joint values, separated with spaces, by using ROS

"""

# Import module
import rospy
from std_msgs.msg import Float32MultiArray

# Meta data
__author__ = 'Norman Marlier'
__copyright__ = 'Copyright 2018'
__credits__ = ['Norman Marlier']
__maintainer__ = "Norman Marlier"
__email__ = "norman.marlier@uliege.be"
__status__ = "Prototype"


def talker():
    pub = rospy.Publisher('controller-lorang', Float32MultiArray, queue_size=10)
    rospy.init_node('controller-talker', anonymous=True)
    rate = rospy.Rate(20)  # 20hz - 50ms

    while not rospy.is_shutdown():
        s = input()
        cmd_input = list(map(int, s.split()))
        print(cmd_input)
        cmd_motors = Float32MultiArray()
        cmd_motors.data = cmd_input

        pub.publish(cmd_motors)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



