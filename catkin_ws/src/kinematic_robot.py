import math
import numpy as np

from matplotlib import pyplot as plt

# Constants
L1 = 1
L2 = 1
L3 = 1


def fkine(angular_pose):
    """
    Forward kinematic solver

    :param angular_pose: The wanted angular position (3 dof) (list)
    :return: the cartesian position related to the angular pose
    """
    # Compute the cartesian coordinates
    x = math.cos(angular_pose[0]) * (L2 * math.cos(angular_pose[1]) + L3 * math.cos(angular_pose[2]))
    y = math.sin(angular_pose[0]) * (L2 * math.cos(angular_pose[1]) + L3 * math.cos(angular_pose[2]))
    z = L1 + L2 * math.sin(angular_pose[1]) + L3 * math.sin(angular_pose[2])

    return x, y, z


def ikine(cartesian_pose, config=True):
    """
    Inverse kinematic solver

    :param cartesian_pose: The wanted cartesian position (3 dof)
    :param config: the configuration of the result :
                   - False : angle_2 > angle_1
                   - True : angle_1 > angle_2
    :return: the angles related to the cartesian pose
    """

    # Theta 1
    theta_1 = math.atan2(cartesian_pose[1], cartesian_pose[0])

    # Intermediate variables
    A = math.sqrt(pow(cartesian_pose[0], 2) + pow(cartesian_pose[1], 2))
    B = cartesian_pose[2] - L1
    a = L2
    b = L3
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

    if config:
        return theta_1, theta_2_1, beta_1

    else:
        return theta_1, theta_2_2, beta_2


def polynome(A, B, C, D, E, F, time_step):
    A = [x * pow(time_step, 5) for x in A]
    B = [x * pow(time_step, 4) for x in B]
    C = [x * pow(time_step, 3) for x in C]
    D = [x * pow(time_step, 2) for x in D]
    E = [x * pow(time_step, 1) for x in E]
    return [a+b+c+d+e+f for a, b, c, d, e, f in zip(A, B, C, D, E, F)]


def polynomial_trajectory(initial, final, number_point):
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
        trajectory.append(polynome(A, B, C, [0, 0, 0], [0, 0, 0], initial, i))
        print('poly', polynome(A, B, C, [0, 0, 0], [0, 0, 0], initial, i))

    return trajectory

traj = polynomial_trajectory([0.3, 0.4, 0.3], [0.5, 0.5, 0.4], 30)
plt.plot(range(30), traj)
plt.show()


