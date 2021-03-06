from sympy import *
from time import time
from mpmath import radians
import tf
from . import inverse_kinematics as ik
from . import forward_kinematics as fk
from . import utils
import logging
import sys

base_logger = logging.getLogger()
base_logger.setLevel(logging.DEBUG)
'''
IK_debug as been moved to this file.
usage is:
    python -m kr210_kinematics <test_id>

with test_id the integer key of the test you want to run.

Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$    roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

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



def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]
    # pdb.set_trace()
    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ##
    # our code needs those unpacked to work.
    pos = [0, 0, 0]
    pos[0] = req.poses[0].position.x
    pos[1] = req.poses[0].position.y
    pos[2] = req.poses[0].position.z

    orient = [0, 0, 0, 0]
    orient[0] = req.poses[0].orientation.x
    orient[1] = req.poses[0].orientation.y
    orient[2] = req.poses[0].orientation.z
    orient[3] = req.poses[0].orientation.w
    ## Insert IK code here!
    print("Building Kinematic Providers.")
    start_time_build_ik = time()
    ik_provider = ik.get_inverse_kinematics()
    end_time_build_ik = time()
    print("Building IK class took: {}".format(end_time_build_ik - start_time_build_ik))
    start_time_evaluate_ik = time()
    joint_values = ik_provider.evaluate_pose(pos, orient)
    end_time_evaluate_ik = time()
    print("Evaluating IK took: {}".format(end_time_evaluate_ik - start_time_evaluate_ik))

    theta1 = joint_values[0]
    theta2 = joint_values[1]
    theta3 = joint_values[2]
    theta4 = joint_values[3]
    theta5 = joint_values[4]
    theta6 = joint_values[5]

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kr210_kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    fk_provider = fk.get_forward_kinematics()
    joints_dict = utils.get_eval_dict(theta_1=theta1,
                                      theta_2=theta2,
                                      theta_3=theta3,
                                      theta_4=theta4,
                                      theta_5=theta5,
                                      theta_6=theta6)
    T_EE = fk_provider.evaluate_transform("T_0_EE", joints_dict)
    T_WC = fk_provider.evaluate_transform("T_0_5", joints_dict)
    ## End your code input for forward kr210_kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [T_WC[0, 3], T_WC[1, 3], T_WC[2, 3]] # <--- Load your calculated WC values in this array
    your_ee = [T_EE[0, 3], T_EE[1, 3], T_EE[2, 3]] # <--- Load your calculated end effector value from your forward kr210_kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\n Wrist position: {}".format(your_wc))
        print ("Wrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nJoint values: {}".format(joints_dict))
    print ("Theta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector position: {}".format(your_ee))
        print ("End effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # Change test case number for different scenarios
    argv = sys.argv
    if len(argv) > 1:
        test_case_number = int(argv[1])
    else:
        test_case_number = 1
    print "test case nb: {}".format(test_case_number)
    test_code(test_cases[test_case_number])
