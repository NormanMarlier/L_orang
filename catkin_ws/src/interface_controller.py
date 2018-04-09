#!/usr/bin/env python

"""

docstring

"""

# Import module
import rospy
from std_msgs.msg import Int32MultiArray


# Meta data
__author__ = 'Norman Marlier'
__copyright__ = 'Copyright 2018'
__credits__ = ['Norman Marlier']
__maintainer__ = "Norman Marlier"
__email__ = "norman.marlier@student.uliege.be"
__status__ = "Prototype"


def talker():
    pub = rospy.Publisher('lorang', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    angle1 = 90
    angle2 = 80
    angle3 = 60
    gripper = 0
    while not rospy.is_shutdown():
		cmd_motors = Int32MultiArray();
		cmd_motors.data = [angle1, angle2, angle3, gripper]
        rospy.loginfo('Change the state of the gripper')
        pub.publish(cmd_motors)
        rate.sleep()

if __name__ == '__main__':
  	
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



