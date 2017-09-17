from mpmath import *
from sympy import *
import numpy as np
from .utils import *
from . import FK
import tf


def get_transform_from_pose(position, orientation):
    angles = tf.transformations.euler_from_quaternion(orientation)
    T_EE = tf.transformations.compose_matrix(angle=angles, translate=position)
    return T_EE


def law_of_cosines(opposite, a, b):
    """
    Returns the angle for the triangle at the opposite of opposite.
    """
    return np.arccos((a**2 + b**2 - opposite**2) / (2 * a * b))



### Inverse position

### set our helper variables
W = T_EE[:3, 3] - (d6 + d7) * T_EE[:3, 2]

w_x = W[0]
w_z = W[2]
D = np.linalg.norm((w_x, w_z))  # only operate on x
B = w_x
E = w_z
C = DH_table[a2]
A_prime = DH_table[d4]
F = DH_table[a3]
A = np.linalg.norm((A_prime, F))

### Run the IK calculation sequence.


theta_1 = np.arctan2(W[2], W[1])

angle_alpha = law_of_cosines(A, C, D)
angle_beta = law_of_cosines(B, E, D)
theta_2 = angle_beta - angle_alpha

angle_gamma = law_of_cosines(D, C, A)
angle_gamma_prime =law_of_cosines(F, A, A_prime)
theta3 = angle_gamma + angle_gamma_prime

### Inverse orientation
joints_values = FK.get_eval_dict(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3)

T = FK.eval_forward_kinematic("T_0_3", joints_values)
R_0_3 = T[:3, :3]
R_3_6 = R_0_3.inv("LU") * T_EE[:3, :3]

tf.transformations.euler_from_matrix(R_3_6)


