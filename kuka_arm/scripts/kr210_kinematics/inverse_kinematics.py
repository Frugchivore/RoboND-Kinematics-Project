from mpmath import *
from sympy import *
import numpy as np
from .utils import *
from .forward_kinematics import  get_forward_kinematics
import tf

_ik_instance = None


def law_of_cosines(opposite, a, b):
    """
    Returns the angle for the triangle at the opposite of opposite.
    """
    return np.arccos((a**2 + b**2 - opposite**2) / (2 * a * b))


def law_of_cosines2(opposite, a, b):
    """
    Returns the angle for the triangle at the opposite of opposite.
    """
    return (a**2 + b**2 - opposite**2) / (2 * a * b)


class InverseKinematics:
    def __init__(self):
        self.forward_kinematics = get_forward_kinematics()

    def evaluate_pose(self, position, orientation):
        angles = tf.transformations.euler_from_quaternion(orientation)
        end_effector_transform = tf.transformations.compose_matrix(angles=angles, translate=position)
        return self.evaluate_transform(end_effector_transform)

    def evaluate_transform(self, end_effector_transform):
        W = end_effector_transform[:3, 3] - (DH_table[d6] + DH_table[d7]) * end_effector_transform[:3, 2]
        W = np.concatenate((W, [0]))

        # Obtain theta_1 from the current EE transform.
        theta_1 = np.arctan2(W[1], W[2])

        # Adjust the transform to position the origin at the origin of joint 2 in base frame orientation.
        joints_values = get_eval_dict(theta_1=theta_1)
        T02 = self.forward_kinematics.evaluate_transform("T_0_2", joints_values)
        T02[:3, :3] = np.eye(3)  # We're not interested in the rotation.

        W02 = np.linalg.inv(T02).dot(W)

        # Set of helper variable to apply the law of cosine.
        w_x = W02[0]
        w_z = W02[2]
        D = np.linalg.norm((w_x, w_z))
        B = w_x
        E = w_z
        C = DH_table[a2]
        A_prime = DH_table[d4]
        F = DH_table[a3]
        A = np.linalg.norm((A_prime, F))

        angle_alpha = law_of_cosines(A, C, D)
        angle_beta = law_of_cosines(B, E, D)
        theta_2 = angle_beta - angle_alpha

        angle_gamma = law_of_cosines(D, C, A)
        # adjustment for the position fo the wrist in Z axis (from base frame)
        angle_gamma_prime = law_of_cosines(F, A, A_prime)
        theta_3 = angle_gamma + angle_gamma_prime - np.pi/2

        ### Inverse orientation
        joints_values = get_eval_dict(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3)

        T = self.forward_kinematics.evaluate_transform("T_0_3", joints_values)
        R_0_3 = T[:3, :3]
        R_3_6 = np.linalg.inv(R_0_3) * T_EE[:3, :3]

        theta_4, theta_5, theta_6 = tf.transformations.euler_from_matrix(R_3_6)
        return theta_1, theta_2, theta_3, theta_4, theta_5, theta_6

# ### Inverse position
# position = [2.1529, 0, 1.9465]
# orientation = [0, -0.00014835, 0, 1]





