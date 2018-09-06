import math
from matplotlib import pyplot as plt


# This class represents a kinematic solver (no dynamic equations)
# This solver is based on geometrical features such as the distances between
# two joints.
# It can be used only for 3 DOF mechanical systems as robotic arms
# with a RRR configuration

# Lorang - born_2 (-10, 120)


class KinematicSolver(object):

    def __init__(self, l1, l2, l3, born_1, born_2, born_3):
        """
        Constructor for the class

        :param l1: distance between the ground of the base and the first joint
        :param l2: distance between the first joint and the second joint
        :param l3: distance between the second joint and the end-effector
        :param born_1: (tuple) min and max angle for the first joint in rad
        :param born_2: (tuple) min and max angle for the second joint in rad
        :param born_3: (tuple) min and max angle for the third joint in rad
        """
        self._L1 = l1
        self._L2 = l2
        self._L3 = l3
        self.born_1_min = born_1[0]
        self.born_1_max = born_1[1]
        self.born_2_min = born_2[0]
        self.born_2_max = born_2[1]
        self.born_3_min = born_3[0]
        self.born_3_max = born_3[1]

    def check_angle(self, angular_pose):
        """
        Check if the angular pose exists
        :param angular_pose: The wanted angular position (3 dof) (list)
        :return: true if ok
                 false if not
        """
        if angular_pose[0] > self.born_1_max or angular_pose[0] < self.born_1_min:
            return False
        if angular_pose[1] > self.born_2_max or angular_pose[1] < self.born_2_min:
            return False
        if angular_pose[2] > self.born_3_max or angular_pose[2] < self.born_3_min:
            return False

        return True

    def fkine(self, angular_pose):
        """
        Forward kinematic solver

        :param angular_pose: The wanted angular position in rad (3 dof) (list)
        :return: the cartesian position related to the angular pose
                 Raise ValueError if the angular_pose is not ok
        """
        # Check the angular pose
        if not self.check_angle(angular_pose):
            raise ValueError

        # Compute the cartesian coordinates
        x = math.cos(angular_pose[0]) * (self._L2 * math.cos(angular_pose[1]) + self._L3 * math.cos(angular_pose[2]))
        y = math.sin(angular_pose[0]) * (self._L2 * math.cos(angular_pose[1]) + self._L3 * math.cos(angular_pose[2]))
        z = self._L1 + self._L2 * math.sin(angular_pose[1]) + self._L3 * math.sin(angular_pose[2])

        return x, y, z

    def ikine(self, cartesian_pose, config=True):
        """
        Inverse kinematic solver

        :param cartesian_pose: The wanted cartesian position (3 dof)
        :param config: the configuration of the result :
                       - False : angle_2 > angle_1
                       - True : angle_1 > angle_2
        :return: the angles related to the cartesian pose in rad
        """

        # Theta 1
        theta_1 = math.atan2(cartesian_pose[1], cartesian_pose[0])

        # Intermediate variables
        A = math.sqrt(pow(cartesian_pose[0], 2) + pow(cartesian_pose[1], 2))
        B = cartesian_pose[2] - self._L1
        a = self._L2
        b = self._L3
        c_alpha = (pow(A, 2) + pow(B, 2) - pow(b, 2) + pow(a, 2)) / (2 * a)
        t_alpha_1 = (B + math.sqrt(pow(A, 2) + pow(B, 2) - pow(c_alpha, 2))) / (A + c_alpha)
        t_alpha_2 = (B - math.sqrt(pow(A, 2) + pow(B, 2) - pow(c_alpha, 2))) / (A + c_alpha)
        beta_1 = math.atan2(B - a * 2 * t_alpha_1 / (1 + pow(t_alpha_1, 2)),
                       A - a * (1 - pow(t_alpha_1, 2)) / (1 + pow(t_alpha_1, 2)))
        beta_2 = math.atan2(B - a * 2 * t_alpha_2 / (1 + pow(t_alpha_2, 2)),
                       B - a * (1 - pow(t_alpha_2, 2)) / (1 + pow(t_alpha_2, 2)))

        # Theta 2
        theta_2_1 = math.atan2(2 * t_alpha_1 / (1 + pow(t_alpha_1, 2)), (1 - pow(t_alpha_1, 2)) / (1 + pow(t_alpha_1, 2)))
        theta_2_2 = math.atan2(2 * t_alpha_2 / (1 + pow(t_alpha_2, 2)), (1 - pow(t_alpha_2, 2)) / (1 + pow(t_alpha_2, 2)))

        # Check the angular poses
        if not self.check_angle([theta_1, theta_2_1, beta_1]):
            raise ValueError
        if not self.check_angle([theta_1, theta_2_2, beta_2]):
            raise ValueError

        if config:
            return theta_1, theta_2_1, beta_1

        else:
            return theta_1, theta_2_2, beta_2

    @staticmethod
    def polynome(A, B, C, D, E, F, time_step):
        A = [x * pow(time_step, 5) for x in A]
        B = [x * pow(time_step, 4) for x in B]
        C = [x * pow(time_step, 3) for x in C]
        D = [x * pow(time_step, 2) for x in D]
        E = [x * pow(time_step, 1) for x in E]
        return [a+b+c+d+e+f for a, b, c, d, e, f in zip(A, B, C, D, E, F)]

    def joint_trajectory(self, initial, final, number_point):
        """

        :param initial:
        :param final:
        :param number_point:
        :return:
        """
        # Polynomial coefficients
        A = ([6 * (y-x) / pow(number_point, 5) for y, x in zip(final, initial)])
        B = ([-15 * (y-x) / pow(number_point, 4) for y, x in zip(final, initial)])
        C = ([10 * (y-x) / pow(number_point, 3) for y, x in zip(final, initial)])

        trajectory = []

        for i in range(number_point):
            trajectory.append(self.polynome(A, B, C, [0, 0, 0], [0, 0, 0], initial, i))

        return trajectory

    @staticmethod
    def show_trajectory(traj):
        """

        :param traj: a valid trajectory (list of 3D lists)
        :return: show the trajectory
        """
        plt.plot(range(len(traj)), traj)
        plt.show()

if __name__ == '__main__':
    # Do some tests
    kine_solver = KinematicSolver(1, 1, 1, [-0.1, 3.14], [-0.1, 3.14], [-0.1, 3.14])

    # Show an inverse calculation
    init_pose_angle = [0, math.pi/4, 0]
    print('Initial configuration - angle ', init_pose_angle[0], ', ', init_pose_angle[1], ', ', init_pose_angle[2])
    init_pose_cart = kine_solver.fkine(init_pose_angle)
    print('Initial configuration - cartesian ', init_pose_cart[0], ', ', init_pose_cart[1], ', ', init_pose_cart[2])
    end_pose_angle = kine_solver.ikine(init_pose_cart)
    print('Finial configuration - angle ', end_pose_angle[0], ', ', end_pose_angle[1], ', ', end_pose_angle[2])

    # Show a joint trajectory
    traj = kine_solver.joint_trajectory(init_pose_angle, [math.pi/4, math.pi/3, math.pi/2], 100)
    kine_solver.show_trajectory(traj)

