from mpmath import *
from sympy import *
import numpy as np
from .utils import *

T_0_1 = create_DH_transform(DH_table, alpha0, a0, d1, q1)
T_1_2 = create_DH_transform(DH_table, alpha1, a1, d2, q2)
T_2_3 = create_DH_transform(DH_table, alpha2, a2, d3, q3)
T_3_4 = create_DH_transform(DH_table, alpha3, a3, d4, q4)
T_4_5 = create_DH_transform(DH_table, alpha4, a4, d5, q5)
T_5_6 = create_DH_transform(DH_table, alpha5, a5, d6, q6)
T_6_7 = create_DH_transform(DH_table, alpha6, a6, d7, q7)

# Correction to orientation of the end effector. (180 Z, -90 Y)
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
              [sin(np.pi), cos(np.pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(-np.pi/2), 0, sin(-np.pi/2), 0],
              [0, 1, 0, 0],
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [0, 0, 0, 1]])

R_corr = simplify(R_z * R_y)

T_0_2 = simplify(T_0_1 * T_1_2)
T_0_3 = simplify(T_0_2 * T_2_3)
T_0_4 = simplify(T_0_3 * T_3_4)
T_0_5 = simplify(T_0_4 * T_4_5)
T_0_6 = simplify(T_0_5 * T_5_6)
T_0_7 = simplify(T_0_6 * T_6_7)


T_0_EE = simplify(T_0_7 * R_corr)

kinematic_dict = {
    "T_0_1": T_0_1,
    "T_0_2": T_0_2,
    "T_0_3": T_0_3,
    "T_0_4": T_0_4,
    "T_0_5": T_0_5,
    "T_0_6": T_0_6,
    "T_0_7": T_0_7,
    "T_0_EE": T_0_EE,
    "T_1_2": T_1_2,
    "T_2_3": T_2_3,
    "T_3_4": T_3_4,
    "T_4_5": T_4_5,
    "T_5_6": T_5_6,
    "T_6_7": T_6_7,
}


def eval_forward_kinematic(tf_key, eval_dict):
    transform = kinematic_dict[tf_key]
    return transform.evalf(subs=eval_dict)


def get_eval_dict(theta_1=None, theta_2=None, theta_3=None, theta_4=None, theta_5=None, theta_6=None):
    return {
        q1: theta_1,
        q2: theta_2,
        q3: theta_3,
        q4: theta_4,
        q5: theta_5,
        q6: theta_6
            }
