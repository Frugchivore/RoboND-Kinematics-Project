#!/usr/bin/env python
"""
Module for the Inverse Kinematics Provider.
To improve on performance it is recommended to get the IK via the static factory
get_inverse_kinematics()
This will insure that Inverse Kinematics is kept as a singleton.
"""


from mpmath import *
from sympy import *
import numpy as np
from .utils import *
from .forward_kinematics import get_forward_kinematics
import tf
import pdb

_ik_instance = None


def inv_law_of_cosines(opposite, a, b):
    """
    Returns the angle for the triangle at the opposite of opposite.
    """
    return acos((a**2 + b**2 - opposite**2) / (2 * a * b))


def law_of_cosines(opposite, a, b):
    """
    Returns the angle for the triangle at the opposite of opposite.
    """
    return (a**2 + b**2 - opposite**2) / (2 * a * b)


def angle_from_cos(cos_value):
    return atan2(cos_value, -sqrt(1-cos_value**2))


class _InverseKinematics:
    def __init__(self):
        self.forward_kinematics = get_forward_kinematics()

    def evaluate_pose(self, position, orientation):
        """
        Evaluate position and orientation and returns the joint angles solving for those.
        :param position: list of size 3, contains the x,y,z position of the end effector.
        :param orientation: list of size 4, contains the x,y,z,w quaternion of the end effector.
        :return: list of size 6 of the joints values.
        """
        position = Matrix(position)
        RRR = tf.transformations.quaternion_matrix(orientation)
        Rrpy = RRR * R_corr

        W = position - DH_table[d7] * Rrpy[:3, 2]


        # Obtain theta_1 from the current EE transform.
        theta_1 = atan2(W[1], W[0])

        # Set of helper variable to apply the law of cosine.
        w_x = W[0] - DH_table[a1] * cos(theta_1)
        w_y = W[1] - DH_table[a1] * sin(theta_1)
        w_z = W[2] - DH_table[d1]

        B = sqrt(w_x**2 + w_y**2)
        E = w_z
        D = sqrt(B**2 + E**2)
        C = DH_table[a2]
        A_prime = DH_table[d4]
        F = DH_table[a3]
        A = sqrt(A_prime**2 + F**2)

        angle_a = inv_law_of_cosines(A, C, D)
        angle_d = inv_law_of_cosines(D, C, A)

        kappa = atan2(E, B)
        theta_2 = pi/2 - (angle_a + kappa)

        adjustment = inv_law_of_cosines(F, A, A_prime)
        theta_3 = pi/2 - (angle_d + adjustment)


        ### Inverse orientation
        joints_values = get_eval_dict(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3)
        R_0_3 = self.forward_kinematics.evaluate_transform("T_0_3", joints_values, False)
        R_3_6 = R_0_3.inv() * Rrpy

        r12 = R_3_6[1, 2]
        cq5 = r12
        r02 = R_3_6[0, 2]
        r22 = R_3_6[2, 2]
        sq5 = simplify(sqrt(r02 ** 2 + r22 ** 2))
        theta_5 = atan2(sq5, cq5)

        theta_4 = atan2(r22, -r02)

        r10 = R_3_6[1, 0]
        r11 = R_3_6[1, 1]
        theta_6 = atan2(-r11, r10)

        # return everything evaluated.
        #return theta_1, theta_2, theta_3, theta_4, theta_5, theta_6

        return theta_1.evalf(), theta_2.evalf(), theta_3.evalf(), theta_4.evalf(), theta_5.evalf(), theta_6.evalf()

def get_inverse_kinematics():
    global _ik_instance
    if _ik_instance is None:
        _ik_instance = _InverseKinematics()
    return _ik_instance
