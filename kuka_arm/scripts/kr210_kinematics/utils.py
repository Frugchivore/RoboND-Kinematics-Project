#!/usr/bin/env python
"""
Set of objects and method used to support the forward and inverse kinematics functionality.
- Denavit Hartenberg table for KR210.
- generic euler angle rotation matrices.
- Method to obtain the joints values dictionary for FK and IK requests.
- All relevant Sympy symbols
"""


from __future__ import division
from mpmath import *
from sympy import *
import numpy as np

# A matrix suitable for any DH homogenous transform
# One can then fix it with a substitution.
alpha_i_1, a_i_1, d_i, theta_i = symbols("alpha_i_1 a_i_1 d_i theta_i")
_DH_HT = Matrix([[cos(theta_i), -sin(theta_i), 0, a_i_1],
                [sin(theta_i)*cos(alpha_i_1), cos(theta_i)*cos(alpha_i_1), -sin(alpha_i_1), -sin(alpha_i_1) * d_i],
                [sin(theta_i)*sin(alpha_i_1), cos(theta_i)*sin(alpha_i_1), cos(alpha_i_1), cos(alpha_i_1) * d_i],
                [0, 0, 0, 1]])




## defines symbols
q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")

alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")

### DH Table for KR210 ###
DH_table = {
    alpha0:     0, a0:      0, d1: 0.75, q1: q1,
    alpha1: -pi/2, a1:   0.35, d2: 0,    q2: q2 - pi/2,
    alpha2:     0, a2:   1.25, d3: 0, q3: q3,
    alpha3: -pi/2, a3: -0.054, d4: 1.50, q4: q4,
    alpha4:  pi/2, a4:      0, d5: 0, q5: q5,
    alpha5: -pi/2, a5:      0, d6: 0, q6: q6,
    alpha6:     0, a6:      0, d7: 0.303, q7: 0
}


def create_dh_transform(dh_table, alpha, a, d, theta):
    subs_dict = {
        alpha_i_1: alpha,
        a_i_1:a,
        d_i:d,
        theta_i:theta
    }
    temp_denavit_hartenberg_transform = _DH_HT.subs(subs_dict)
    return temp_denavit_hartenberg_transform.subs(dh_table)


def get_eval_dict(theta_1=None, theta_2=None, theta_3=None, theta_4=None, theta_5=None, theta_6=None):
    return {
        q1: theta_1,
        q2: theta_2,
        q3: theta_3,
        q4: theta_4,
        q5: theta_5,
        q6: theta_6
    }

### Euler angles rotations ####
x, y, z = symbols("x y z")

R_x = Matrix([[1, 0, 0, 0],
              [0, cos(x), -sin(x), 0],
              [0, sin(x), cos(x), 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(y), 0, sin(y), 0],
              [0, 1, 0, 0],
              [-sin(y), 0, cos(y), 0],
              [0, 0, 0, 1]])

R_z = Matrix([[cos(z), -sin(z), 0, 0],
              [sin(z), cos(z), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

# Correction to orientation of the end effector. (180 Z, -90 Y), z=pi, y=-np.pi/2
R_corr = simplify(R_z.subs({z: radians(180)}) * R_y.subs({y: radians(-90)}))
