from mpmath import *
from sympy import *
import numpy as np
from .utils import *

# Creating the class is very expensive. Store it here as a singleton.
# Use
_fk_instance = None


class _ForwardKinematics:
    def __init__(self):
        # Joint to Joint transforms
        T_0_1 = create_dh_transform(DH_table, alpha0, a0, d1, q1)
        T_1_2 = create_dh_transform(DH_table, alpha1, a1, d2, q2)
        T_2_3 = create_dh_transform(DH_table, alpha2, a2, d3, q3)
        T_3_4 = create_dh_transform(DH_table, alpha3, a3, d4, q4)
        T_4_5 = create_dh_transform(DH_table, alpha4, a4, d5, q5)
        T_5_6 = create_dh_transform(DH_table, alpha5, a5, d6, q6)
        T_6_7 = create_dh_transform(DH_table, alpha6, a6, d7, q7)

        # Base to Joint transforms
        T_0_2 = simplify(T_0_1 * T_1_2)
        T_0_3 = simplify(T_0_2 * T_2_3)
        T_0_4 = simplify(T_0_3 * T_3_4)
        T_0_5 = simplify(T_0_4 * T_4_5)
        T_0_6 = simplify(T_0_5 * T_5_6)
        T_0_7 = simplify(T_0_6 * T_6_7)

        # Final full transform
        T_0_EE = simplify(T_0_7 * R_corr)

        self.kinematic_dict = {
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

    def evaluate_transform(self, tf_key, joints_dict):
        transform = self.kinematic_dict[tf_key]
        return matrix2numpy(transform.evalf(subs=joints_dict), dtype=np.float64)


def get_forward_kinematics():
    global _fk_instance
    if _fk_instance is None:
        _fk_instance = _ForwardKinematics()
    return _fk_instance

