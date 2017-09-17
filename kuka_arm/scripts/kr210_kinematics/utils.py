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
