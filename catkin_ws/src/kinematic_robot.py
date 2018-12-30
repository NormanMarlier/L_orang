import math
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation

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

        l1 > 0 | l2 > 0 | l3 > 0

        :param l1: (>= 0) distance between the ground of the base and the first joint (z axis)
        :param l2: (>= 0) distance between the first joint and the second joint
        :param l3: (>= 0) distance between the second joint and the end-effector
        :param born_1: (tuple) min and max angle for the first joint in rad
        :param born_2: (tuple) min and max angle for the second joint in rad
        :param born_3: (tuple) min and max angle for the third joint in rad
        """
        if l1 < 0 or l2 < 0 or l3 < 0:
            raise ValueError('a joint distance is negative')
        if (l1 == 0) & (l2 == 0) & (l3 == 0):
            raise ValueError('the joint distances are equal to 0')

        self._L1 = l1
        self._L2 = l2
        self._L3 = l3
        self.born_1_min = born_1[0]
        self.born_1_max = born_1[1]
        self.born_2_min = born_2[0]
        self.born_2_max = born_2[1]
        self.born_3_min = born_3[0]
        self.born_3_max = born_3[1]

    def __check_angle(self, angular_pose):
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
        if not self.__check_angle(angular_pose):
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
        A = float(math.sqrt(pow(cartesian_pose[0], 2) + pow(cartesian_pose[1], 2)))
        B = float(cartesian_pose[2] - self._L1)
        a = float(self._L2)
        b = float(self._L3)
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
        if not self.__check_angle([theta_1, theta_2_1, beta_1]) and config:
            print("Configuration 1 is not ok")
            raise ValueError
        if not self.__check_angle([theta_1, theta_2_2, beta_2]) and (not config):
            print("Configuration 2 is not ok")
            raise ValueError

        if config:
            return theta_1, theta_2_1, beta_1

        else:
            return theta_1, theta_2_2, beta_2

    @staticmethod
    def __polynome(A, B, C, D, E, F, eval_pt):
        """
        Return a fifth degree polynome
        :param A: coeff for 5 degree
        :param B: coeff for 4 degree
        :param C: coeff for 3 degree
        :param D: coeff for 2 degree
        :param E: coeff for 1 degree
        :param F: constante
        :param eval_pt: the value where the polynome is evaluated
        :return: A*(eval_pt)^5 + B*(eval_pt)^4 + C*(eval_pt)^3 + D*(eval_pt)^2 + E*(eval_pt) + F
        """
        A = [x * pow(eval_pt, 5) for x in A]
        B = [x * pow(eval_pt, 4) for x in B]
        C = [x * pow(eval_pt, 3) for x in C]
        D = [x * pow(eval_pt, 2) for x in D]
        E = [x * pow(eval_pt, 1) for x in E]
        return [a+b+c+d+e+f for a, b, c, d, e, f in zip(A, B, C, D, E, F)]

    def __start_to_end_joint_trajectory(self, initial, final, number_point):
        """
        Generate a valid joint trajectory by using polynomial interpolation

        :param initial: Initial joint pose (must be valid)
        :param final: Final joint pose (must be valid)
        :param number_point: The number of points for the trajectory
        :param points: Intermediate points (empty list by default)
        :return: A joint trajectory
        """
        # Check the initial and the final pose
        if not self.__check_angle(initial):
            print("Initial pose is not valid")
            raise ValueError
        if not self.__check_angle(final):
            print("Final pose is not valid")
            raise ValueError

        # Polynomial coefficients
        A = ([6 * (y-x) / pow(number_point, 5) for y, x in zip(final, initial)])
        B = ([-15 * (y-x) / pow(number_point, 4) for y, x in zip(final, initial)])
        C = ([10 * (y-x) / pow(number_point, 3) for y, x in zip(final, initial)])

        trajectory = []

        for i in range(number_point):
            trajectory.append(self.__polynome(A, B, C, [0, 0, 0], [0, 0, 0], initial, i))

        return trajectory

    def __start_to_end_linear_trajectory(self, initial, final, number_point):
        """
        Generate a joint trajectory by using linear cartesian interpolation
        /!\ Singularity can occur /!\

        :param initial: Initial cartesian pose
        :param final: Final cartesian pose
        :param number_point: The number of points for the trajectory
        :param points: Intermediate points (empty list by default)
        :return: A joint trajectory based on cartesian pose
        """
        trajectory = []

        init_pose = np.array(initial)
        final_pose = np.array(final)
        delta_pose = final_pose - init_pose

        # Built the trajectory
        for i in range(number_point):
            # cartesian coordinate
            x = init_pose + (delta_pose/number_point)*i
            try:
                new_point = self.ikine(list(x))
            except ValueError:
                raise ValueError
            else:
                trajectory.append(new_point)

        return trajectory

    def joint_trajectory(self, points, number_points):
        """

        :param points: (2x3 min) list of angle points
        :param number_points: (>=1)The number of points for the trajectory
        :return: a valid trajectory
        """
        # Check points - size (2,3) min
        if np.shape(points)[0] < 2 or np.shape(points)[1] is not 3:
            print("Not the good dimension")
            raise TypeError

        # Iterate on points
        trajectory = np.zeros((number_points*(np.shape(points)[0]-1), np.shape(points)[1]))
        for i in range(len(points)-1):
            trajectory[i*number_points:(i+1)*number_points, :] = self.__start_to_end_joint_trajectory(points[i][:], points[i+1][:], number_points)

        # Return the trajectory
        return trajectory

    def linear_trajectory(self, points, number_points):
        """

        :param points: (2x3 min) list of cartesian points
        :param number_points: (>=1)The number of points for the trajectory
        :return: a valid trajectory
        """
        # Check points - size (2,3) min
        if np.shape(points)[0] < 2 or np.shape(points)[1] is not 3:
            print("Not the good dimension")
            raise TypeError

        # Iterate on points
        trajectory = np.zeros((number_points*(np.shape(points)[0]-1), np.shape(points)[1]))

        for i in range(len(points)-1):
            trajectory[i*number_points:(i+1)*number_points, :] = self.__start_to_end_linear_trajectory(points[i][:], points[i+1][:], number_points)

        # Return the trajectory
        return trajectory

    @staticmethod
    def show_angle(traj):
        """

        :param traj: a valid trajectory (list of 3D lists)
        :return: show the angles
        """
        trajectory = np.array(traj)
        for i in range(0, np.shape(traj)[1]):
            plt.plot(range(len(traj)), trajectory[:, i], label=["$\theta_{0}$".format(i)])
        plt.legend(shadow=True)
        plt.show()

    def animate_trajectory(self, traj, record=False, file_name='animation.mp4'):
        """

        :param traj: A valid trajectory
        :param record: True if the animation is recorded. False (by default) otherwise
        :param file_name:
        :return:
        """
        # ------------------------------------------------------------
        # set up figure and animation
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-3, 3), ylim=(-3, 3))
        ax.grid()

        line1, = ax.plot([], [], '-o', lw=2)
        line2, = ax.plot([], [], 'r-', lw=2)
        x_traj = []
        y_traj = []
        plt.plot()

        def init():
            """initialize animation"""
            line1.set_data([], [])
            line2.set_data([], [])
            return line1, line2,

        def animate(i):
            """perform animation step"""
            # X coordinate for the two articulations
            thisx = [0, self._L2 * math.cos(traj[i][1]), self._L2 * math.cos(traj[i][1]) + self._L3 * math.cos(traj[i][2])]
            thisz = [self._L1, self._L1 + self._L2 * math.sin(traj[i][1]), self._L1 + self._L2 * math.sin(traj[i][1]) + self._L3 * math.sin(traj[i][2])]
            line1.set_data(thisx, thisz)
            x_traj.append(thisx[2])
            y_traj.append(thisz[2])
            line2.set_data(x_traj, y_traj)
            return line1, line2,

        ani = animation.FuncAnimation(fig, animate, np.arange(1, len(traj)),
                                      interval=10, blit=True, init_func=init, repeat=False)

        # save the animation as an mp4.  This requires ffmpeg or mencoder to be
        # installed.  The extra_args ensure that the x264 codec is used, so that
        # the video can be embedded in html5.  You may need to adjust this for
        # your system: for more information, see
        # http://matplotlib.sourceforge.net/api/animation_api.html
        if record:
            ani.save(file_name, fps=30, extra_args=['-vcodec', 'libx264'])

        plt.show()


if __name__ == '__main__':
    # Do some tests
    kine_solver = KinematicSolver(1, 1, 1, [-3.14, 3.14], [-3.14, 3.14], [-3.14, 3.14])

    # Show an inverse calculation
    init_pose_angle = [0., math.pi/4, 0.]
    print('Initial configuration - angle ', init_pose_angle)
    init_pose_cart = kine_solver.fkine(init_pose_angle)
    print('Initial configuration - cartesian ', init_pose_cart)
    intermediate_pose_angle = [0., math.pi/3, math.pi/4]
    intermediate_pose_cart = kine_solver.fkine(intermediate_pose_angle)
    end_pose_angle = [0., math.pi/4, math.pi/4]
    print('Final configuration - angle ', end_pose_angle)
    end_pose_cart = kine_solver.fkine(end_pose_angle)
    print('Final configuration - cartesian ', end_pose_cart)

    # Show a joint trajectory
    traj = kine_solver.linear_trajectory([init_pose_cart, intermediate_pose_cart, end_pose_cart], 100)
    kine_solver.show_angle(traj)
    kine_solver.animate_trajectory(traj)



