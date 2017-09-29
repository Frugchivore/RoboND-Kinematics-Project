## Project: Robotic Arm Pick &amp; Place
____

This implementation of the Pick &amp; Place project was ran on a Ubuntu 16.04 device with the following specifications:

    Memory: 7.7 GiB
    Processor: Intel Core i7-3537U CPU @ 2.00GHz x 4
    Graphics: Intel Ivybridge Mobile
    OS Type: 64-bit



The objective of the project was to calculate and implement in code the inverse kinematics
of an industrial manipulator, in closed-form using the Geometric method.




### Kinematic Analysis
___

#### Denavit-Hartenberg frame assignment


To make the frame assignment, we tried to maximise the number of zero parameters in the table. An interesting result was that frame 1 was not placed at the joint, but at the same level as frame 2. This allowed to set $d_2$ to 0 and reach frame 2 via a simple translation.
In addition to that, and to allow us to decouple the inverse kinematics, we put frame 4, 5, 6 all centered on joint 5. With $d_7$ providing the offset necessary to reach the end effector.

In more details the offset were obtain as follow, using the world/urdf frame as reference for the axis:

* $d_1$ : {z offset of joint 1} + {z offset of joint 2} -> 0.33 + 0.42 = 0.75
* $a_1$ : {x offset of joint 1} -> 0.35
* $a_2$ : {z offset of joint 3} -> 1.25
* $a_3$ : {z offset of joint 4} -> -0.054
* $d_4$ : {x offset of joint 4} + {x offset of joint 5}  -> 0.96 + 0.54 = 1.50
* $d_7$ : {x offset of joint 6} + {x offset of gripper joint} -> 0.193 + 0.11 = 0.303


This results in the following Denavit Hartenberg table:

|i         |$\alpha_{i-1}$|$a_{i-1}$|$d_i$  |$\theta_i$ |
|:--------:|:------------:|:-------:|:----: |:--------: |
|$T_0^1$   |   $0$        |   $0$   |$0.75$ |   $q_1$   |
|$T_1^2$   |   $-\pi/2$   |$0.35$   |$0$    |$q_2 - \pi/2$|
|$T_2^3$   |   $0$        |$1.25$   |$0$    |   $q_3$   |
|$T_3^4$   |   $-\pi/2$   |$-0.054$ |$1.50$ |   $q_4$   |
|$T_4^5$   |   $\pi/2$    |   $0$   |       |   $q_5$   |
|$T_5^6$   |   $-\pi/2$   |   $0$   |       |   $q_6$   |
|$T_6^7$   |   $0$        |   $0$   |$0.303$|   $0$     |

#### Transforms

The generic Denavit Hartenberg transform takes the following form:
$$\left[\begin{matrix}\cos{\left (\theta_{i} \right )} & - \sin{\left (\theta_{i} \right )} & 0 & a_{i 1}\\\sin{\left (\theta_{i} \right )} \cos{\left (\alpha_{i 1} \right )} & \cos{\left (\alpha_{i 1} \right )} \cos{\left (\theta_{i} \right )} & - \sin{\left (\alpha_{i 1} \right )} & - d_{i} \sin{\left (\alpha_{i 1} \right )}\\\sin{\left (\alpha_{i 1} \right )} \sin{\left (\theta_{i} \right )} & \sin{\left (\alpha_{i 1} \right )} \cos{\left (\theta_{i} \right )} & \cos{\left (\alpha_{i 1} \right )} & d_{i} \cos{\left (\alpha_{i 1} \right )}\\0 & 0 & 0 & 1\end{matrix}\right]$$

By applying the correct substitution to this generic transform we can obtain the transforms about each joint.

1. $$T_0^1=\left[\begin{matrix}\cos{\left (q_{1} \right )} & - \sin{\left (q_{1} \right )} & 0 & 0\\\sin{\left (q_{1} \right )} & \cos{\left (q_{1} \right )} & 0 & 0\\0 & 0 & 1 & 0.75\\0 & 0 & 0 & 1\end{matrix}\right]$$

2. $$T_1^2=\left[\begin{matrix}\sin{\left (q_{2} \right )} & \cos{\left (q_{2} \right )} & 0 & 0.35\\0 & 0 & 1 & 0\\\cos{\left (q_{2} \right )} & - \sin{\left (q_{2} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]$$

3. $$T_2^3=\left[\begin{matrix}\cos{\left (q_{3} \right )} & - \sin{\left (q_{3} \right )} & 0 & 1.25\\\sin{\left (q_{3} \right )} & \cos{\left (q_{3} \right )} & 0 & 0\\0 & 0 & 1 & 0\\0 & 0 & 0 & 1\end{matrix}\right]$$

4. $$T_3^4=\left[\begin{matrix}\cos{\left (q_{4} \right )} & - \sin{\left (q_{4} \right )} & 0 & -0.054\\0 & 0 & 1 & 1.5\\- \sin{\left (q_{4} \right )} & - \cos{\left (q_{4} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]$$

5. $$T_4^5=\left[\begin{matrix}\cos{\left (q_{1} \right )} & - \sin{\left (q_{1} \right )} & 0 & 0\\\sin{\left (q_{1} \right )} & \cos{\left (q_{1} \right )} & 0 & 0\\0 & 0 & 1 & 0.75\\0 & 0 & 0 & 1\end{matrix}\right]$$

6. $$T_5^6=\left[\begin{matrix}\cos{\left (q_{6} \right )} & - \sin{\left (q_{6} \right )} & 0 & 0\\0 & 0 & 1 & 0\\- \sin{\left (q_{6} \right )} & - \cos{\left (q_{6} \right )} & 0 & 0\\0 & 0 & 0 & 1\end{matrix}\right]$$

7. $$T_6^7=\left[\begin{matrix}1 & 0 & 0 & 0\\0 & 1 & 0 & 0\\0 & 0 & 1 & 0.303\\0 & 0 & 0 & 1\end{matrix}\right]$$

Note that $T_6^7$ is used to translate the transform from the wrist center to the correct end-effector location.


Note that we need to apply a rotational correction to our kinematic chain to account for the difference in orientation between the manipulator's end-effector and the frames affixed to it via DH.
This rotational correction consists of a rotation about the Z axis of $\pi$ degrees and a rotation about the Y axis of $-\pi/2$.
    $$R_{corr}=\left[\begin{matrix}-6.12323399573677 \cdot 10^{-17} & -1.22464679914735 \cdot 10^{-16} & 1.0 & 0\\7.49879891330929 \cdot 10^{-33} & -1.0 & -1.22464679914735 \cdot 10^{-16} & 0\\1.0 & 0 & 6.12323399573677 \cdot 10^{-17} & 0\\0 & 0 & 0 & 1\end{matrix}\right]$$

From those transforms, we can obtain the forward kinematics transform from the base link to the end effector. This can be done via matrix multiplication:
$$T_0^{EE} = T_0^1 \times T_1^2  \times T_2^3 \times T_3^4 \times T_4^5 \times T_5^6 \times T_6^7 \times R_{corr} $$

For clarity, and to leave this document uncluttered we will not display this transform in full.

Another transform that would prove of interest to the reader is the same EE transform, from the point of view of the pose of the end effector. Let us assume the pose can be represented by two vectors, one for the position, and one for the orientation:
    
    Pose:
        position: { p_x, p_y, p_z }
        orientation: { r, p, y }
        
We should immediately point out that using roll, pitch, yaw to represent the orientation may be helpful to the engineer, but poses certain problem for the robot itself. To represent a full rotation matrix, we need 16 numbers. On a long kinematic chain, this increases the risk of numerical noise accumulating as well as memory usage. To alleviate that, it is usually preferable in production to use Quaternion to represent the rotation via the vector ${x, y, z, w}$. At the design stage, we can work with a symbolic computation library such as Sympy, to reduce the impact of noise.

Going back to our problem, a full Euler angle xyz intrinsic rotation composed with a translation will take the form:
$$T_0^{EE}=\left[\begin{matrix}\cos{\left (p \right )} \cos{\left (y \right )} & \sin{\left (p \right )} \sin{\left (r \right )} \cos{\left (y \right )} - \sin{\left (y \right )} \cos{\left (r \right )} & \sin{\left (p \right )} \cos{\left (r \right )} \cos{\left (y \right )} + \sin{\left (r \right )} \sin{\left (y \right )} & p_{x}\\\sin{\left (y \right )} \cos{\left (p \right )} & \sin{\left (p \right )} \sin{\left (r \right )} \sin{\left (y \right )} + \cos{\left (r \right )} \cos{\left (y \right )} & \sin{\left (p \right )} \sin{\left (y \right )} \cos{\left (r \right )} - \sin{\left (r \right )} \cos{\left (y \right )} & p_{y}\\- \sin{\left (p \right )} & \sin{\left (r \right )} \cos{\left (p \right )} & \cos{\left (p \right )} \cos{\left (r \right )} & p_{z}\\0 & 0 & 0 & 1\end{matrix}\right]
$$






### Project Implementation:
___
#### Structure of the project:

Modification were made to the files provided by Udacity and additional files were added to make the structure of the project more consistent.
All the files and their content is described below:

- *IK_server.py* [new]: Contains the ROS node used to provide the IKCalculation service.

- *kr210_kinematics/\__main\__.py* [from udacity]: Contains the IK_debug.py code modified to work with the rest of the package. Allows to call the package more easily from the command line via 'python -m kr210_kinematics'
- *kr210_kinematics/forward_kinematics.py* [new]: Contains the code to generate all relevant transforms for the Forward Kinematics of our manipulator.

- *kr210_kinematics/inverse_kinematics.py* [new]: Provides the actual implementation of the IK, used by the IK_server to execute the IK calculations. 

- *kr210_kinematics/utils.py* [new]: Contains the Sympy symbols and helper methods used in the package. Also contains the Denavit Hartenberg table as a dictionary of symbols. 


##### Building the Forward Kinematics:

The utils module stores a generic DH Transform as well as the DH table.
The method __utils.create_dh_transform(dh_table, alpha, a, d, theta)__ can be used to generate the transforms about each joint.

All the relevant FK code is stored in the forward_kinematics module.
We store the forward kinematics into a class \_ForwardKinematics that builds all the transforms in its constructor.
Because this is a costl operation, we keep the class hidden and make it accessible only through the static factory get_forward_kinematics(). This makes it de facto a singleton and allows us to ensure the cost of building the FK transforms is paid only once in the live of the application. This significantly speeds up the FK and IK computations. After a warm up of about 1-2 seconds to instantiate the transforms, calculating FK or IK takes no more than 0.1 seconds.

The class provides the method __evaluate_transform(self, tf_key, joints_dict, as_array=True)__ to compute a given transform either as Sympy matrix or numpy array.

##### Building the Inverse Kinematics:

As IK depends on FK, we use the same logic and wrap the IK code into a singleton, \_InverseKinematics class.
This class provides the method __evaluate_pose(self, position, orientation)__, taking a position and orientation as quaternion and returning the joints values for this pose. The details of the calculation are identical to the demonstration made in the preceding section. but given its key role as part of the project, we provide its code below:
```python

   def evaluate_pose(self, position, orientation):
        """
        Evaluate position and orientation and returns the joint angles solving for those.
        :param position: list of size 3, contains the x,y,z position of the end effector.
        :param orientation: list of size 4, contains the x,y,z,w quaternion of the end effector.
        :return: list of size 6 of the joints values.
        """
        position = Matrix(position)
        RRR = tf.transformations.quaternion_matrix(orientation)
        Rrpy = RRR * R_corr

        W = position - DH_table[d7] * Rrpy[:3, 2]


        # Obtain theta_1 from the current EE transform.
        theta_1 = atan2(W[1], W[0])

        # Set of helper variable to apply the law of cosine.
        w_x = W[0] - DH_table[a1] * cos(theta_1)
        w_y = W[1] - DH_table[a1] * sin(theta_1)
        w_z = W[2] - DH_table[d1]

        B = sqrt(w_x**2 + w_y**2)
        E = w_z
        D = sqrt(B**2 + E**2)
        C = DH_table[a2]
        A_prime = DH_table[d4]
        F = DH_table[a3]
        A = sqrt(A_prime**2 + F**2)

        angle_a = inv_law_of_cosines(A, C, D)
        angle_d = inv_law_of_cosines(D, C, A)

        kappa = atan2(E, B)
        theta_2 = pi/2 - (angle_a + kappa)

        adjustment = inv_law_of_cosines(F, A, A_prime)
        theta_3 = pi/2 - (angle_d + adjustment)


        ### Inverse orientation
        joints_values = get_eval_dict(theta_1=theta_1, theta_2=theta_2, theta_3=theta_3)
        R_0_3 = self.forward_kinematics.evaluate_transform("T_0_3", joints_values, False)
        R_3_6 = R_0_3.inv() * Rrpy

        r12 = R_3_6[1, 2]
        cq5 = r12
        r02 = R_3_6[0, 2]
        r22 = R_3_6[2, 2]
        sq5 = simplify(sqrt(r02 ** 2 + r22 ** 2))
        theta_5 = atan2(sq5, cq5)

        theta_4 = atan2(r22, -r02)

        r10 = R_3_6[1, 0]
        r11 = R_3_6[1, 1]
        theta_6 = atan2(-r11, r10)

        return theta_1.evalf(), theta_2.evalf(), theta_3.evalf(), theta_4.evalf(), theta_5.evalf(), theta_6.evalf()

```

##### Serving the IK:

IK_server.py has been refactored into a class with the interface __run()__ and __handle_calculate_IK(req)__.
This refactoring allows us to call __get_inverse_kinematics()__ in the constructor and make sure that all IK request are made at full speed. We added diagnostic code to allow the user to see the poses and orientation sent as input and the sames values returned by the FK using the joints value returned by IK.
This diagnostic can be controlled via the ROS parameters "__debug__" and "__debug_freq__" that we added to the launch file for __inverse_kinematics.launch__.

An example log from the diagnostic is seen below:

TODO ADD LOG

Finally, you can see one sequence of the pick and place in the video found in kuka_arm/media/pick_and_place.ogv.

##### Discussion:

Once the inverse kinematics calculation is correct, the manipulator will follow the path provided very tightly. Error on test data was 0, and the video clearly show that the path and orientations are good.

Issues encountered in the course of the implementation were:
- forgetting to import \__division\__ from the future package, causing results to be off.
- Using numpy objects instead of sympy ones, causing numerical noise. ex. using np.pi instead of sympy.pi.

Interestingly, symplifying the transforms did not improve the speed of the FK or IK in any meaningful way. Yet it severely increases the time required to build the transforms initially. A possible reason is that by the time the transforms get evaluated, all symbols have been replaced by their values and they are easy to evaluate. Therefore we do not benefit from the simplified symbolic formulas.


##### Possible extensions & improvements:

- We can see in trajectory_sampler.cpp that the path is calculated using a Rapidly-Exploring Random Tree algotithm, provided by MoveIt. The core idea of RRT is rather straightforward, the real difficulty being in the implementation of the collision detection system used to reject nodes in the path. An interesting additional project could be to implement the RRT to construct the manipulators path ourselves.


- The mouvements of the manipulator are rather slow, it would be interesting to see the impact of a higher joints velocity. A trade will be necessary between faster mouvement and error in the resulting pose. This would be a good opportunity to add a simple controller on top of our IK to help correct the pose and joints values errors.


- It can be interesting to solve the KR210 IK problem using the algebraic method discussed in J. Craig's Introduction to Robotics. Maybe, using symbolic computation, one might find some patterns that could be reusable to  obtain the IK solution for a spherical wrist robot, from its DH table in a semi-automated fashion.




