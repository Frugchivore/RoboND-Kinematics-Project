#!/usr/bin/env python

"""
Module for the Forward Kinematics Provider.
To improve on performance it is recommended to get the FK via the static factory
get_forward_kinematics()
This will insure that Forward Kinematics is kept as a singleton.
"""

__author__ = "Ndil-Adjim Sou Ngadoy"

from sympy import matrix2numpy
from .utils import *
import logging

"""
This modules 
"""

# Creating the class is very expensive. Store it here as a singleton.
# Use
_fk_instance = None

class _ForwardKinematics:
    def __init__(self):
        # Joint to Joint transforms
        logging.debug("Creating DH Transforms")
        T_0_1 = create_dh_transform(DH_table, alpha0, a0, d1, q1)
        T_1_2 = create_dh_transform(DH_table, alpha1, a1, d2, q2)
        T_2_3 = create_dh_transform(DH_table, alpha2, a2, d3, q3)
        T_3_4 = create_dh_transform(DH_table, alpha3, a3, d4, q4)
        T_4_5 = create_dh_transform(DH_table, alpha4, a4, d5, q5)
        T_5_6 = create_dh_transform(DH_table, alpha5, a5, d6, q6)
        T_6_7 = create_dh_transform(DH_table, alpha6, a6, d7, q7)

        # Base to Joint transforms
        logging.debug("Creating Base to Joint Transforms")
        T_0_2 = T_0_1 * T_1_2
        T_0_3 = T_0_2 * T_2_3
        T_0_4 = T_0_3 * T_3_4
        T_0_5 = T_0_4 * T_4_5
        T_0_6 = T_0_5 * T_5_6
        T_0_7 = T_0_6 * T_6_7

        # Final full transform
        logging.debug("Creating end effector Transform")
        T_0_EE = T_0_7 * R_corr


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
        logging.debug("All Transforms done.")

    def evaluate_transform(self, tf_key, joints_dict, as_array=True):
        """
        Computes the forward kinematics for a given chain of joints and returns the transform.
        :param tf_key: name of the transform to evaluate.
        :param joints_dict: dict of joints value. use get_eval_dict to create it in the correct format.
        :param as_array: Flag to return either the numpy array of the transform or a sympy matrix.
        :return: T, transform evaluated with the joints value.

        usage:
            joints_value = utils.get_eval_dict(theta_1=1, theta_2=0.22)
            fk_provider = get_forward_kinematics()
            fk_provider.evaluate_transform("T_0_2", joints_value)
        """
        transform = self.kinematic_dict[tf_key]
        T = transform.evalf(subs=joints_dict)
        if as_array:
            T = matrix2numpy(T, dtype=np.float64)
        return T


def get_forward_kinematics():
    """
    Static factory to obtain the instance of ForwardKinematics.
    :return: single instance of ForwardKinematics.
    """
    global _fk_instance
    if _fk_instance is None:
        _fk_instance = _ForwardKinematics()
    return _fk_instance

