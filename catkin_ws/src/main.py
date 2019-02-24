### This script is the main program of the Lorang controller
###
### It uses RRRSolver class to generate valid trajectories to send to the low level controller (Arduino board)
### Test if everything works well



# Import module
import rospy
from kinematic_robot import RRRSolver
from robot_controller import RobotController


def talker():
    # ROS
    rospy.init_node('controller-talker-lorang', anonymous=True)
    rate = rospy.Rate(5)  # 1Hz = 1s

    # Robot Controller
    robot = RobotController()
 
    # Kinematic solver
    kine_solver = RRRSolver(0.095, 0.15, 0.15, [-180., 180.], [0., 180.], [0., 180.])
    kine_solver.calibration([0., 180., 110.], [1, -1, -1], [-180., 180.], [80., 150.], [80., 135.])
    
    # Rectangle
    pts_1 = [0.14, 0, 0.2]
    pts_2 = [0.22, 0, 0.25]
    pts_3 = [0.14, 0, 0.25]
    pts_4 = [0.22, 0, 0.2]
    
    # Compute trajectory
    traj = kine_solver.linear_trajectory([pts_1, pts_3, pts_2, pts_4, pts_1], 20)
    counter = 0

    while not rospy.is_shutdown():
        # Publish the angles
        robot.move_to(traj[counter])

        counter += 1
        if counter >= len(traj):
            counter = len(traj) - 1

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
