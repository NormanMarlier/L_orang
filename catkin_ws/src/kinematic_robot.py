import math
import abc
import numpy as np

from scipy.spatial import ConvexHull
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3

# This class represents a implementation of a kinematic solver (no dynamic equations)
# This solver is based on geometrical features such as the distances between
# two joints.
# KinematicSolver is an abstract class. Different solvers can be derived from it


class KinematicSolver(abc.ABC):

    @abc.abstractmethod
    def check_angle(self, angular_pose):
        """
        Check if the angular pose exists
        :param angular_pose: The wanted angular position (3 dof) (list)
        :return: true if ok
                 false if not
        """

    @abc.abstractmethod
    def calibration(self, calibration_pose, calibration_direction, born_1, born_2, born_3):
        """
        Calibrate the born of the solver coordinate system.
        Compute the relation between the device coordinate system and the solver coordinate system.
        /!\ In degrees /!\
        Allows the transformations between the hardware and the software

        :param calibration_pose: The values of the angles in the solver coordinate system
        :param calibration_direction: The way the values changes = the rate (value = {-1, 1})
        :param born_1: (tuple) min and max angle for the first joint in degrees
        :param born_2: (tuple) min and max angle for the second joint in degrees
        :param born_3: (tuple) min and max angle for the third joint in degrees
        """

    @abc.abstractmethod
    def to_device_coordinate(self, solver_pose):
        """
        Change the values from the solver coordinate system to the device coordinate system

        :param solver_pose: A pose from the solver coordinate system (in rad)
        :return: 'solver_pose' with respect to the device coordinate system
        """

    @abc.abstractmethod
    def to_solver_coordinate(self, device_pose):
        """
        Change the values from the device coordinate system to the solver coordinate system
        /!\ In degrees /!\

        :param device_pose: A pose from the device coordinate system
        :return: 'device_pose' with respect to the solver coordinate system
        """

    @abc.abstractmethod
    def fkine(self, angular_pose):
        """
        Forward kinematic solver

        :param angular_pose: The wanted angular position in rad (3 dof) (list)
        :return: the cartesian position related to the angular pose
                 Raise ValueError if the angular_pose is not ok
        """

    @abc.abstractmethod
    def ikine(self, cartesian_pose, config=True):
        """
        Inverse kinematic solver

        :param cartesian_pose: The wanted cartesian position (3 dof)
        :param config: the configuration of the result :
                       - False : angle_2 > angle_1
                       - True : angle_1 > angle_2
        :return: the angles related to the cartesian pose in rad
        """

    @staticmethod
    def polynome(A, B, C, D, E, F, eval_pt):
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

    @abc.abstractmethod
    def start_to_end_joint_trajectory(self, initial, final, number_point):
        """
        Generate a valid joint trajectory by using polynomial interpolation

        :param initial: Initial joint pose (must be valid)
        :param final: Final joint pose (must be valid)
        :param number_point: The number of points for the trajectory
        :return: A joint trajectory
        """

    @abc.abstractmethod
    def start_to_end_linear_trajectory(self, initial, final, number_point):
        """
        Generate a joint trajectory by using linear cartesian interpolation
        /!\ Singularity can occur /!\

        :param initial: Initial cartesian pose
        :param final: Final cartesian pose
        :param number_point: The number of points for the trajectory
        :return: A joint trajectory based on cartesian pose
        """

    @abc.abstractmethod
    def joint_trajectory(self, points, number_points):
        """

        :param points: (2x3 min) list of angle points
        :param number_points: (>=1)The number of points for the trajectory
        :return: a valid trajectory
        """

    @abc.abstractmethod
    def linear_trajectory(self, points, number_points):
        """

        :param points: (2x3 min) list of cartesian points
        :param number_points: (>=1)The number of points for the trajectory
        :return: a valid trajectory
        """

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

    @abc.abstractmethod
    def animate_2d_trajectory(self, traj, record=False, file_name='2d_animation.mp4'):
        """

        :param traj: A valid trajectory
        :param record: True if the animation is recorded. False (by default) otherwise
        :param file_name:
        :return: an animation of the trajectory in 2D (x-z plane)
        """

    @abc.abstractmethod
    def animate_3d_trajectory(self, traj, record=False, file_name='3d_animation.mp4'):
        """
        Animate a 3D trajectory and can save it into mp4 file

        :param traj: A valid trajectory
        :param record: True if the animation is recorded. False (by default) otherwise
        :param file_name:
        :return: an animation of the trajectory in 3D
        """

# RRRSolver is a Kinematic Solver
# It can be used only for 3 DOF mechanical systems as robotic arms
# with a RRR configuration
# /!\ Be careful that the angles are given with respect to the horizontal line /!\
#
# Example for Lorang :
# Lorang - born_2 (-10, 120)
# L1 =0.095, L2 = 0.15, L3 = 0.15


class RRRSolver(KinematicSolver):

    def __init__(self, l1, l2, l3, born_1, born_2, born_3):
        """
        Constructor for the class

        l1 >= 0 & l2 > 0 & l3 > 0

        :param l1: (>= 0) distance between the ground of the base and the first joint (z axis)
        :param l2: (> 0) distance between the first joint and the second joint
        :param l3: (> 0) distance between the second joint and the end-effector
        :param born_1: (tuple) min and max angle for the first joint in degrees
        :param born_2: (tuple) min and max angle for the second joint in degrees
        :param born_3: (tuple) min and max angle for the third joint in degrees
        """
        if l1 < 0 or l2 < 0 or l3 < 0:
            raise ValueError('a joint distance is negative')
        if (l2 == 0) | (l3 == 0):
            raise ValueError('A joint distance is equal to 0')

        # Geometrical parameters
        self.__L1 = l1
        self.__L2 = l2
        self.__L3 = l3

        # Born
        self.born_1_min = math.radians(born_1[0])
        self.born_1_max = math.radians(born_1[1])
        self.born_2_min = math.radians(born_2[0])
        self.born_2_max = math.radians(born_2[1])
        self.born_3_min = math.radians(born_3[0])
        self.born_3_max = math.radians(born_3[1])

        # Solver system coordinate parameters
        self.__delta_1 = 0
        self.__delta_2 = 0
        self.__delta_3 = 0
        self.__rate_1 = 1
        self.__rate_2 = 1
        self.__rate_3 = 1

        # Cartesian space
        self.domain_r = []
        self.domain_z = []
        for angle_1 in np.linspace(self.born_2_min, self.born_2_max, 1000, endpoint=True):
            for angle_2 in np.linspace(self.born_3_min, self.born_3_max, 1000, endpoint=True):
                self.domain_r.append(self.__L2 * math.cos(angle_1) + self.__L3 * math.cos(angle_2))
                self.domain_z.append(self.__L1 + self.__L2 * math.sin(angle_1) + self.__L3 * math.sin(angle_2))
        self.cartesian_domain = np.array([self.domain_r, self.domain_z]).T


    def check_angle(self, angular_pose):

        if angular_pose[0] > self.born_1_max or angular_pose[0] < self.born_1_min:
            return False
        if angular_pose[1] > self.born_2_max or angular_pose[1] < self.born_2_min:
            return False
        if angular_pose[2] > self.born_3_max or angular_pose[2] < self.born_3_min:
            return False

        return True

    def calibration(self, calibration_pose, calibration_direction, born_1, born_2, born_3):
        """
        The solver coordinate system is defined as :
        theta 1 = 0, y = 0, z > 0 and -born_x < x < born_x
        theta 2 and 3 start from horizontal line
        """
        # Check the values of 'calibration_direction'
        if (calibration_direction[0] is not 1) and (calibration_direction[0] is not -1):
            raise ValueError("-1 or 1 are expected")
        if (calibration_direction[1] is not 1) and (calibration_direction[1] is not -1):
            raise ValueError("-1 or 1 are expected")
        if (calibration_direction[2] is not 1) and (calibration_direction[2] is not -1):
            raise ValueError("-1 or 1 are expected")

        self.__delta_1 = calibration_pose[0]
        self.__delta_2 = calibration_pose[1]
        self.__delta_3 = calibration_pose[2]
        self.__rate_1 = calibration_direction[0]
        self.__rate_2 = calibration_direction[1]
        self.__rate_3 = calibration_direction[2]

        # Update the born
        self.born_1_min, self.born_2_min, self.born_3_min = self.to_solver_coordinate([born_1[0],
                                                                                       born_2[0],
                                                                                       born_3[0]])
        self.born_1_max, self.born_2_max, self.born_3_max = self.to_solver_coordinate([born_1[1],
                                                                                       born_2[1],
                                                                                       born_3[1]])

        # Update
        if self.born_1_min > self.born_1_max:
            tmp = self.born_1_min
            self.born_1_min = self.born_1_max
            self.born_1_max =tmp
        if self.born_2_min > self.born_2_max:
            tmp = self.born_2_min
            self.born_2_min = self.born_2_max
            self.born_2_max =tmp
        if self.born_3_min > self.born_3_max:
            tmp = self.born_3_min
            self.born_3_min = self.born_3_max
            self.born_3_max =tmp

        # Update cartesian space
        self.domain_r = []
        self.domain_z = []
        for angle_1 in np.linspace(self.born_2_min, self.born_2_max, 1000, endpoint=True):
            for angle_2 in np.linspace(self.born_3_min, self.born_3_max, 1000, endpoint=True):
                self.domain_r.append(self.__L2 * math.cos(angle_1) + self.__L3 * math.cos(angle_2))
                self.domain_z.append(self.__L1 + self.__L2 * math.sin(angle_1) + self.__L3 * math.sin(angle_2))

        self.cartesian_domain = np.array([self.domain_r, self.domain_z]).T

    def to_device_coordinate(self, solver_pose):

        dof_1 = self.__rate_1 * math.degrees(solver_pose[0]) - self.__rate_1 * self.__delta_1
        dof_2 = self.__rate_2 * math.degrees(solver_pose[1]) - self.__rate_2 * self.__delta_2
        dof_3 = self.__rate_3 * math.degrees(solver_pose[2]) - self.__rate_3 * self.__delta_3

        return [dof_1, dof_2, dof_3]

    def to_solver_coordinate(self, device_pose):

        dof_1 = math.radians(self.__rate_1 * device_pose[0] + self.__delta_1)
        dof_2 = math.radians(self.__rate_2 * device_pose[1] + self.__delta_2)
        dof_3 = math.radians(self.__rate_3 * device_pose[2] + self.__delta_3)

        return [dof_1, dof_2, dof_3]

    def fkine(self, angular_pose):

        # Check the angular pose and show the point if not
        if not self.check_angle(angular_pose):
            raise ValueError('The initial position is not possible')

        # Compute the cartesian coordinates
        x = math.cos(angular_pose[0]) * (self.__L2 * math.cos(angular_pose[1]) + self.__L3 * math.cos(angular_pose[2]))
        y = math.sin(angular_pose[0]) * (self.__L2 * math.cos(angular_pose[1]) + self.__L3 * math.cos(angular_pose[2]))
        z = self.__L1 + self.__L2 * math.sin(angular_pose[1]) + self.__L3 * math.sin(angular_pose[2])


        return x, y, z

    def ikine(self, cartesian_pose, config=True):

        # Theta 1
        theta_1 = math.atan2(cartesian_pose[1], cartesian_pose[0])

        # Intermediate variables
        A = math.sqrt(pow(cartesian_pose[0], 2) + pow(cartesian_pose[1], 2))
        B = cartesian_pose[2] - self.__L1
        a = self.__L2
        b = self.__L3
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
        if not self.check_angle([theta_1, theta_2_1, beta_1]) and config:
            raise ValueError("Configuration 1 can't be achieved")
        if not self.check_angle([theta_1, theta_2_2, beta_2]) and (not config):
            raise ValueError("Configuration 2 can't be achieved")

        if config:
            return theta_1, theta_2_1, beta_1

        else:
            return theta_1, theta_2_2, beta_2

    def start_to_end_joint_trajectory(self, initial, final, number_point):

        # Check the initial and the final pose
        if not self.check_angle(initial):
            raise ValueError("Initial pose is not valid")
        if not self.check_angle(final):
            raise ValueError("Final pose is not valid")

        # Polynomial coefficients
        A = ([6 * (y-x) / pow(number_point, 5) for y, x in zip(final, initial)])
        B = ([-15 * (y-x) / pow(number_point, 4) for y, x in zip(final, initial)])
        C = ([10 * (y-x) / pow(number_point, 3) for y, x in zip(final, initial)])

        trajectory = []

        for i in range(number_point):
            trajectory.append(self.polynome(A, B, C, [0, 0, 0], [0, 0, 0], initial, i))

        return trajectory

    def start_to_end_linear_trajectory(self, initial, final, number_point):

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

        # Check points - size (2,3) min
        if np.shape(points)[0] < 2 or np.shape(points)[1] is not 3:
            raise TypeError("Not the right dimension")
        if number_points < 1:
            raise ValueError("Too few number of points")

        # Iterate on points
        trajectory = np.zeros((number_points*(np.shape(points)[0]-1), np.shape(points)[1]))
        for i in range(len(points)-1):
            trajectory[i*number_points:(i+1)*number_points, :] = self.start_to_end_joint_trajectory(points[i][:], points[i+1][:], number_points)

        # Return the trajectory
        return trajectory

    def linear_trajectory(self, points, number_points):

        # Check points - size (2,3) min
        if np.shape(points)[0] < 2 or np.shape(points)[1] is not 3:
            raise TypeError("Not the right dimension")
        if number_points < 1:
            raise ValueError("Too few number of points")

        # Iterate on points
        trajectory = np.zeros((number_points*(np.shape(points)[0]-1), np.shape(points)[1]))

        for i in range(len(points)-1):
            trajectory[i*number_points:(i+1)*number_points, :] = self.start_to_end_linear_trajectory(points[i][:], points[i+1][:], number_points)

        # Return the trajectory
        return trajectory

    def animate_2d_trajectory(self, traj, record=False, file_name='animation.mp4'):
        # ------------------------------------------------------------
        # set up figure and animation
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-.5, .5), ylim=(-.5, .5))
        ax.grid()

        self.show_cartesian_space(False)

        line1, = ax.plot([], [], 'k-o', lw=2)
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
            thisx = [0, self.__L2 * math.cos(traj[i][1]), self.__L2 * math.cos(traj[i][1]) + self.__L3 * math.cos(traj[i][2])]
            thisz = [self.__L1, self.__L1 + self.__L2 * math.sin(traj[i][1]), self.__L1 + self.__L2 * math.sin(traj[i][1]) + self.__L3 * math.sin(traj[i][2])]
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

    def animate_3d_trajectory(self, traj, record=False, file_name='3d_animation.mp4'):

        # Attaching 3D axis to the figure
        fig = plt.figure()
        ax = p3.Axes3D(fig)

        # Setting the axes properties
        x_lim = float(self.__L1 + self.__L2)
        y_lim = x_lim
        z_lim = x_lim + float(self.__L3)
        ax.set_xlim3d([-x_lim, x_lim])
        ax.set_xlabel('X')

        ax.set_ylim3d([-y_lim, y_lim])
        ax.set_ylabel('Y')

        ax.set_zlim3d([-x_lim, z_lim])
        ax.set_zlabel('Z')

        # Lines
        line1, = ax.plot([], [], [], '-o', lw=2)
        line2, = ax.plot([], [], [], 'r-', lw=2)
        x_traj = []
        y_traj = []
        z_traj = []

        def init():
            """initialize animation"""
            line1.set_data([], [])
            line2.set_data([], [])
            return line1, line2,

        def animate(i):
            """perform animation step"""
            # X coordinate for the two articulations
            thisx = [0, math.cos(traj[i][0])*self.__L2 * math.cos(traj[i][1]),
                     math.cos(traj[i][0])*(self.__L2 * math.cos(traj[i][1]) + self.__L3 * math.cos(traj[i][2]))]
            thisy = [0, math.sin(traj[i][0]) * (self.__L2 * math.cos(traj[i][1]) + self.__L3 * math.cos(traj[i][2])),
                     math.sin(traj[i][0]) * (self.__L2 * math.cos(traj[i][1]) + self.__L3 * math.cos(traj[i][2]))]
            thisz = [self.__L1, self.__L1 + self.__L2 * math.sin(traj[i][1]),
                     self.__L1 + self.__L2 * math.sin(traj[i][1]) + self.__L3 * math.sin(traj[i][2])]
            line1.set_data(thisx, thisy)
            line1.set_3d_properties(thisz)
            x_traj.append(thisx[2])
            y_traj.append(thisy[2])
            z_traj.append(thisz[2])
            line2.set_data(x_traj, y_traj)
            line2.set_3d_properties(z_traj)
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

    def show_cartesian_space(self, show=True):
        """
        Show in blue the cartesian space
        :return:
        """
        plt.plot(self.cartesian_domain[:, 0], self.cartesian_domain[:, 1])
        if show:
            plt.show()

    def show_points(self, points):
        """
        Show point(s) in red with respect to the cartesian space of the Solver
        :param points: a no-empty list of 3D points (x, y, z)
        :return: plot the points
        """
        #  and Z axis
        r_points = []
        z_points = []

        # TODO : no loop
        for i in range(len(points)):
            r_points.append(math.sqrt(points[i][0]**2 + points[i][1]**2))
            z_points.append(points[i][2])

        self.show_cartesian_space(show=False)
        plt.plot(r_points, z_points, 'ro')
        plt.show()


if __name__ == '__main__':
    # Do some tests
    kine_solver = RRRSolver(0.095, 0.15, 0.15, [-180., 180.], [0., 180.], [0., 180])

    # Calibration
    kine_solver.calibration([0, 180., 110.], [1, -1, -1], [-180., 180.], [80., 150.], [80., 135])

    # Rectangle
    pts_1 = [0.2, 0, 0.17]
    pts_2 = [0.25, 0, 0.2]
    pts_3 = [0.2, 0, 0.2]
    pts_4 = [.25, 0, 0.17]

    kine_solver.show_points([pts_1, pts_2, pts_3, pts_4])

    # Limits
    #lim_theta_1 = kine_solver.to_solver_coordinate([0, 80, 80])
    #lim_theta_2 = kine_solver.to_solver_coordinate([0, 150, 135])
    #print(lim_theta_1)
    #print(lim_theta_2)
    #pt_1 = kine_solver.to_solver_coordinate([0, 150., 100.])
    #pt_2 = kine_solver.to_solver_coordinate([0, 100., 110.])
    #pt_1 = [0., math.pi/3, -math.pi/8]
    #pt_2 = [0., math.pi/2, math.pi/8]
    # Try inverse kinematic


    # Show a joint trajectory
    traj = kine_solver.linear_trajectory([pts_4, pts_2, pts_3, pts_1, pts_4], 100)
    kine_solver.animate_2d_trajectory(traj)
    #pt_1 = kine_solver.ikine(pts_1)
    #pt_2 = kine_solver.ikine(pts_2)
    #pt_3 = kine_solver.ikine(pts_3)
    #pt_4 = kine_solver.ikine(pts_4)





