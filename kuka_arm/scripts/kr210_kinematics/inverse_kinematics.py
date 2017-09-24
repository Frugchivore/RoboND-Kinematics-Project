from mpmath import *
from sympy import *
import numpy as np
from .utils import *
from .forward_kinematics import  get_forward_kinematics
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
        R_3_6 = R_0_3.inv("LU") * Rrpy

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
        return theta_1, theta_2, theta_3, theta_4, theta_5, theta_6

        # return theta_1.evalf(), theta_2.evalf(), theta_3.evalf(), theta_4.evalf(), theta_5.evalf(), theta_6.evalf()

def get_inverse_kinematics():
    global _ik_instance
    if _ik_instance is None:
        _ik_instance = _InverseKinematics()
    return _ik_instance

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}
