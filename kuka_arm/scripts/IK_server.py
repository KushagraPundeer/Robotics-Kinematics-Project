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
import numpy as np

def create_matrix(alpha, d, a, q):  
    T = Matrix([[           cos(q),           -sin(q),           0,             a],   
                [cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                [sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                [                0,                 0,           0,             1]])
    return T

def rotation_x(q):
    r_x = Matrix([[1,      0,       0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q),  cos(q)]])
    return r_x
        
def rotation_y(q):
    r_y = Matrix([[cos(q), 0, sin(q)],
                  [     0, 1,      0],
                  [-sin(q), 0, cos(q)]])
    return r_y

def rotation_z(q):
    r_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q),  cos(q), 0],
                  [     0,       0, 1]])
    return r_z

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Conversion b/w radians and degrees
        rtd = 180 / pi
        dtr = pi / 180
		
        ### Your FK code here
        # Create symbols
        # Define joint angles - theta
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        # Define twist angles
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
        # Define link lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # Define link offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 

        # Modified DH parameters
        s = {alpha0:     0, d1:  0.75, a0:      0, q1:        q1,
             alpha1: -pi/2, d2:     0, a1:   0.35, q2: q2 - pi/2,
             alpha2:     0, d3:     0, a2:   1.25, q3:        q3,
             alpha3: -pi/2, d4:  1.50, a3: -0.054, q4:        q4,
             alpha4:  pi/2, d5:     0, a4:      0, q5:        q5,
             alpha5: -pi/2, d6:     0, a5:      0, q6:        q6,
             alpha6:     0, d7: 0.303, a6:      0, q7:         0}

        # Define Homogenous Transformation - base_link to gripper
        T0_1 = create_matrix(alpha0, d1, a0, q1)
        T0_1 = T0_1.subs(s)

        T1_2 = create_matrix(alpha1, d2, a1, q2)
        T1_2 = T1_2.subs(s)

        T2_3 = create_matrix(alpha2, d3, a2, q3)
        T2_3 = T2_3.subs(s)

        #T3_4 = create_matrix(alpha3, d4, a3, q4)
        #T3_4 = T3_4.subs(s)

        #T4_5 = create_matrix(alpha4, d5, a4, q5)
        #T4_5 = T4_5.subs(s)

        #T5_6 = create_matrix(alpha5, d6, a5, q6)
        #T5_6 = T5_6.subs(s)

        #T6_G = create_matrix(alpha6, d7, a6, q7)
        #T6_G = T6_G.subs(s)

        # Composition of Homogenous transformation
        T0_2 = simplify(T0_1 * T1_2)    # base_link to link_2
        T0_3 = simplify(T0_2 * T2_3)    # base_link to link_3
        #T0_4 = simplify(T0_3 * T3_4)    # base_link to link_4
        #T0_5 = simplify(T0_4 * T4_5)    # base_link to link_5
        #T0_6 = simplify(T0_5 * T5_6)    # base_link to link_6
        #T0_G = simplify(T0_6 * T6_G)    # base_link to gripper_link


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # EF location
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # EF orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            # Inverse Kinematics code
            # Compensate for rotation discrepancy between DH paramets and Gazebo
            R_z = rotation_z(pi)
            R_y = rotation_y(-pi/2)
            R_corr = R_z * R_y
            Rrpy = rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll) * R_corr
            nx = Rrpy[0, 2]
            ny = Rrpy[1, 2]
            nz = Rrpy[2, 2]

            # Calculate Wrist center
            WC_x = px - (0.303 + 0) * nx
            WC_y = py - (0.303 + 0) * ny
            WC_z = pz - (0.303 + 0) * nz
            theta1 = atan2(WC_y, WC_x).evalf()                       
            a1, a2, a3 = 0.35, 1.25, -0.054
            d1, d4 = 0.75, 1.5

            # Calculate Geometry and Trignometry b/w joint_2, joint_3 & WC
            # Center of Joint_2: Point_A | Center of Joint_3: Point_B
            line_AB = a2 
            line_BC = sqrt(a3**2 + d4**2) 
            
            # Hypotenuse b/w WC_x and WC_y: hypo_WC_xy
            hypo_WC_xy = sqrt(WC_x**2+WC_y**2)
            line_CA = sqrt((hypo_WC_xy - a1)**2 + (WC_z - d1)**2) 
            angle_A = acos((line_AB**2 + line_CA**2 - line_BC**2) / (2 * line_AB * line_CA))
            angle_B = acos((line_AB**2 + line_BC**2 - line_CA**2) / (2 * line_AB * line_BC))
            gamma = atan2(WC_z - d1, hypo_WC_xy - a1)
            beta = atan2(d4, -a3)
            theta2 = pi/2 - angle_A - gamma
            theta3 = -(angle_B - beta)
            
            # Extract rotational matrix from homogenous transformation matrix
            R0_3 = T0_3[:3,:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv('LU') * Rrpy      
            
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])

            if sin(theta5) < 0:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)


        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
