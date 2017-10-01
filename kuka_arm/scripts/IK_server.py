#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import time

# Inverse and forward kinematics logic is stored in this package.
from kr210_kinematics.inverse_kinematics import get_inverse_kinematics
from kr210_kinematics.utils import get_eval_dict

# Flag to allow for diagnostic output.
DEBUG = False
DEBUG_FREQ = 5


class IK_server:
    """
    Server acting as bridge between the IK provider and the rest of the system.
    """
    def __init__(self):
        # Building the provider is a costly operation, save the cost by building it before any request is served.
        self.ik_provider = get_inverse_kinematics()

    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1
        else:

            do_debug = rospy.get_param("/trajectory_sampler/debug")
            if do_debug:
                debug_freq = rospy.get_param("/trajectory_sampler/debug_freq")

            # Initialize service response
            start_time = time.time()
            joint_trajectory_list = []
            for x in xrange(0, len(req.poses)):
                # IK code starts here
                joint_trajectory_point = JointTrajectoryPoint()

                position = [0, 0, 0]
                position[0] = req.poses[x].position.x
                position[1] = req.poses[x].position.y
                position[2] = req.poses[x].position.z

                orientation = [0, 0, 0, 0]
                orientation[0] = req.poses[x].orientation.x
                orientation[1] = req.poses[x].orientation.y
                orientation[2] = req.poses[x].orientation.z
                orientation[3] = req.poses[x].orientation.w
                ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            #
            # Calculate joint angles using Geometric IK method
            #
            #
                ###

                # Populate response for the IK request
                thetas = self.ik_provider.evaluate_pose(position, orientation)
                joint_trajectory_point.positions = thetas

                if do_debug and x % debug_freq == 0:
                    rospy.loginfo("DIAGNOSTIC at step {}".format(x))
                    joints_values = get_eval_dict(theta_1=thetas[0], theta_2=thetas[1], theta_3=thetas[2], theta_4=thetas[3], theta_5=thetas[4], theta_6=thetas[5])
                    EE = self.ik_provider.forward_kinematics.evaluate_transform("T_0_EE", joints_values, True)
                    rospy.loginfo("pos in: {}".format(req.poses[x].position))
                    rospy.loginfo("pos out: x: {}, y: {}, z: {}".format(EE[0,3], EE[1,3], EE[2,3]))
                    rospy.loginfo("orient in: {}".format(req.poses[x].orientation))
                    q = tf.transformations.quaternion_from_matrix(EE)
                    rospy.loginfo("orient out: x {}, y: {}, z: {}, w: {}".format(*q))

                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)

    def run(self):
        # initialize node and declare calculate_ik service
        rospy.init_node('IK_server')
        s = rospy.Service('calculate_ik', CalculateIK, self.handle_calculate_IK)
        print "Ready to receive an IK request"
        rospy.spin()

if __name__ == "__main__":
    IK_server().run()
