from mpmath import *
from sympy import *
import numpy
from . import utils


## defines symbols
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5,alpha6 = symbols("alpha0:7")

### DH Table for KR210 ###
DH_table = {
    alpha0:     0, a0:      0, d1: 0.75,
    alpha1: -pi/2, a1:   0.35, d2: 0, q2: q2 - pi/2,
    alpha2:     0, a2:   1.25, d3: 0,
    alpha3: -pi/2, a3: -0.054, d4: 1.50,
    alpha4:  pi/2, a4:      0, d5: 0,
    alpha5: -pi/2, a5:      0, d6: 0,
    alpha6:     0, a6:      0, d7: 0.303, q7: 0
}

T_0_1 = utils.create_DH_transform(DH_table, alpha0, a0, d1, q1)
T_1_2 = utils.create_DH_transform(DH_table, alpha1, a1, d2, q2)
T_2_3 = utils.create_DH_transform(DH_table, alpha2, a2, d3, q3)
T_3_4 = utils.create_DH_transform(DH_table, alpha3, a3, d4, q4)
T_4_5 = utils.create_DH_transform(DH_table, alpha4, a4, d5, q5)
T_5_6 = utils.create_DH_transform(DH_table, alpha5, a5, d6, q6)
T_6_7 = utils.create_DH_transform(DH_table, alpha6, a6, d7, q7)

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

eval_dict = {
    q1: None,
    q2: None,
    q3: None,
    q4: None,
    q5: None,
    q6: None
}

eval_dict = {
    q1: -0.99,
    q2: 0,
    q3: 0,
    q4: 0,
    q5: 0,
    q6: 0
}