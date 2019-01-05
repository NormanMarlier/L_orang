#!/usr/bin/env python

"""

docstring

"""

# Import module
import rospy
import math
from std_msgs.msg import Float32MultiArray
from src.kinematic_robot import RRRSolver

# Meta data
__author__ = 'Norman Marlier'
__copyright__ = 'Copyright 2018'
__credits__ = ['Norman Marlier']
__maintainer__ = "Norman Marlier"
__email__ = "norman.marlier@uliege.be"
__status__ = "Prototype"


def talker():
    pub = rospy.Publisher('lorang', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(15) # 15hz
    #  Compute a trajectory
    #
    gripper = 0
    k = 0
    while not rospy.is_shutdown():
        cmd_motors = Float32MultiArray()
        cmd_motors.data = [traj[k][0], math.degrees(traj[k][1]), traj[k][2], gripper]
        print(math.degrees(traj[k][1]))
        pub.publish(cmd_motors)
        gripper = 1 - gripper
        k += 1
        if k == 100:
            k = 0
        print('k =', k)
        rate.sleep()

if __name__ == '__main__':
  	
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



