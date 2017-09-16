from mpmath import *
from sympy import *
import numpy as np

# A matrix suitable for any DH homogenous transform
# One can then fix it with a substitution.
alpha_i_1, a_i_1, d_i, theta_i = symbols("alpha_i_1 a_i_1 d_i theta_i")
_DH_HT = Matrix([[cos(theta_i), -sin(theta_i), 0, a_i_1],
                [sin(theta_i)*cos(alpha_i_1), cos(theta_i)*cos(alpha_i_1), -sin(alpha_i_1), -sin(alpha_i_1)*d_i],
                [sin(theta_i)*sin(alpha_i_1), cos(theta_i)*sin(alpha_i_1), cos(alpha_i_1), cos(alpha_i_1) * d_i],
                [0, 0, 0, 1]])


def create_DH_transform(DH_table, alpha, a, d, theta):
    subs_dict = {
        alpha_i_1: alpha,
        a_i_1:a,
        d_i:d,
        theta_i:theta
    }
    temp_DH_HT = _DH_HT.subs(subs_dict)
    return temp_DH_HT.subs(DH_table)
