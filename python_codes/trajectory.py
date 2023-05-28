#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May 25 01:02:01 2023

@author: mehradhq
"""
import math
import numpy as np
import matplotlib.pyplot as plt

# length of index finger links:
    
# proximal phalanx
MP = 39.8
# intermediate phalanx
PD = 22.4
# distal phalanx
DE = 15.8

# Define an empty list to store the x and y coordinates.
# this is for the plotting the positions of the end-effector from the initial position to the target destination.
x_coordinates = []
y_coordinates = []

max_iterations = 10000  # Set the maximum number of iterations. depending on the computing power this can be changed.
tolerance = 0.01  # Set the tolerance for convergence. depending on the computing power this can be changed.

# Set the ranges for joint variables
# these ranges have been given in the report.
theta_M_range = np.array([-math.pi / 3, math.pi / 3])
theta_P_range = np.array([-2 * math.pi / 3, 0])
theta_D_range = np.array([-4 * math.pi / 6, 0])

# Jacobian matrix for the index finger
def jacobian_inverse(theta1, theta2):
    jacobian = np.array([
    [
        -39.8 * math.sin(theta1) - 22.4 * math.sin(theta1 + theta2) - 15.8 * math.sin(theta1 + (5/3) * theta2),
        -22.4 * math.sin(theta1 + theta2) - (5/3) * 15.8 * math.sin(theta1 + (5/3) * theta2)
    ],
    [
        39.8 * math.cos(theta1) + 22.4 * math.cos(theta1 + theta2) + 15.8 * math.cos(theta1 + (5/3) * theta2),
        22.4 * math.cos(theta1 + theta2) + (5/3) * 15.8 * math.cos(theta1 + (5/3) * theta2)
    ]
])
    
    # Compute the Moore-Penrose pseudoinverse of jacobian_inverse
    jacobian_inverse = np.linalg.pinv(jacobian)
    return jacobian_inverse

def diff_to_target(theta1, theta2, target):
    forward_kinematics = np.array([
        math.cos(theta1) * MP + math.cos(theta1 + theta2) * PD + math.cos(theta1 + (5/3) * theta2) * DE,
        math.sin(theta1) * MP + math.sin(theta1 + theta2) * PD + math.sin(theta1 + (5/3) * theta2) * DE
    ])
    diff = target - forward_kinematics
    return diff

# initial guess for joint variables. This can be changed by the user.
theta_M = -0.1439013 
theta_P = -2.03578803
# joint variable array
initial_joint_angles = np.array([theta_M, theta_P])

# target destination. This can be changed by the user.
target_pose = np.array([[13,-18],[14,-18],[15,-18],[16,-18],[17,-18],[18,-18]])

def inverse_kinematics(max_iterations, tolerance, initial_joint, target):
    joint_angles = initial_joint.copy()
    
    # checking if the joint variables are in the given range.
    # given the written program they should be, but this is just for verification.
    assert joint_angles[0] >= theta_M_range[0]
    assert joint_angles[0] <= theta_M_range[1]
    assert joint_angles[1] >= theta_P_range[0]
    assert joint_angles[1] <= theta_P_range[1]
    
    for iteration in range(max_iterations):
        # Check if theta_M is within the valid range
        # if the joint M is forced to go out of its motion range it will resist.
        if joint_angles[0] < -math.pi / 3:
            joint_angles[0] = -math.pi / 3
        elif joint_angles[0] > math.pi / 3:
            joint_angles[0] = math.pi / 3

        # Check if theta_P is within the valid range
        # if the joint P is forced to go out of its motion range it will resist.
        if joint_angles[1] < -2 * math.pi / 3:
            joint_angles[1] = -2 * math.pi / 3
        elif joint_angles[1] > 0:
            joint_angles[1] = 0

        # Forward Kinematics: Compute the end effector pose using current joint angles
        # Calculate the error between the target pose and the current end effector pose
        target_difference = diff_to_target(joint_angles[0], joint_angles[1], target)
        forward_kinematics_res = np.array([
            math.cos(joint_angles[0]) * MP + math.cos(joint_angles[0] + joint_angles[1]) * PD +
            math.cos(joint_angles[0] + (5/3) * joint_angles[1]) * DE,
            math.sin(joint_angles[0]) * MP + math.sin(joint_angles[0] + joint_angles[1]) * PD +
            math.sin(joint_angles[0] + (5/3) * joint_angles[1]) * DE
        ])
        x_coordinates.append(forward_kinematics_res[0])
        y_coordinates.append(forward_kinematics_res[1])
        
        if (iteration<10):
        # Create a new plot for each iteration
            plt.figure()
            
            # Visualize the robotic arm for the current iteration
            plt.plot([0, math.cos(joint_angles[0]) * MP, math.cos(joint_angles[0]) * MP +
                      math.cos(joint_angles[0] + joint_angles[1]) * PD, forward_kinematics_res[0]],
                     [0, math.sin(joint_angles[0]) * MP, math.sin(joint_angles[0]) * MP +
                      math.sin(joint_angles[0] + joint_angles[1]) * PD, forward_kinematics_res[1]],
                     'o-')
    
            # Number the vertices with a sequence from 1 to n
            plt.text(0, 0, '1', color='red')
            plt.text(math.cos(joint_angles[0]) * MP, math.sin(joint_angles[0]) * MP, '2', color='red')
            plt.text(math.cos(joint_angles[0]) * MP + math.cos(joint_angles[0] + joint_angles[1]) * PD,
                     math.sin(joint_angles[0]) * MP + math.sin(joint_angles[0] + joint_angles[1]) * PD, '3', color='red')
            plt.text(forward_kinematics_res[0], forward_kinematics_res[1], str(iteration + 1), color='red')
    
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title(f'Robotic Arm Iteration {iteration + 1}')
            plt.grid(True)
            # saves the image in the given directory. Change it accordingly.
            #plt.savefig(f'/Users/mehradhq/Downloads/Robotics/img{iteration + 1}.png')
        if (iteration >= max_iterations - 1):
            print("Solution not found after", max_iterations, "iterations.")
            return [joint_angles,  np.array([
                 math.cos(joint_angles[0]) * MP + math.cos(joint_angles[0] + joint_angles[1]) * PD + math.cos(joint_angles[0] + (5/3) * joint_angles[1]) * DE,
                 math.sin(joint_angles[0]) * MP + math.sin(joint_angles[0] + joint_angles[1]) * PD + math.sin(joint_angles[0] + (5/3) * joint_angles[1]) * DE
             ])]
        if math.sqrt(target_difference[0]**2 + target_difference[1]**2) < tolerance:
            print("Solution found after", iteration, "iterations.")
            return [joint_angles,  np.array([
                 math.cos(joint_angles[0]) * MP + math.cos(joint_angles[0] + joint_angles[1]) * PD + math.cos(joint_angles[0] + (5/3) * joint_angles[1]) * DE,
                 math.sin(joint_angles[0]) * MP + math.sin(joint_angles[0] + joint_angles[1]) * PD + math.sin(joint_angles[0] + (5/3) * joint_angles[1]) * DE
             ])]

        # Calculate the Jacobian matrix
        # Compute the pseudo-inverse of the Jacobian matrix
        jac_inv = jacobian_inverse(joint_angles[0], joint_angles[1])
        joint_angles += jac_inv.dot(target_difference)

    return None

for i in target_pose:
    result = inverse_kinematics(max_iterations, tolerance, initial_joint_angles, i)

    if result is not None:
        print("the final joint variables are: ", result[0])
        print("the final position of the end-effector of the index finger is: ", result[1])
        # Plot the points and connect them with directed edges
        plt.figure()
        plt.xlim(10, 20)  # Adjust the x-axis limits as desired
        plt.ylim(-20, -15)  # Adjust the y-axis limits as desired
        plt.plot(x_coordinates, y_coordinates, '-r', marker='o')  # Use '-r' to plot red edges with markers
        # Number the vertices with a sequence from 1 to n
        prev_x, prev_y = x_coordinates[0], y_coordinates[0]
        plt.text(prev_x, prev_y, str(1), color='red')
        for i, (x, y) in enumerate(zip(x_coordinates, y_coordinates), start=1):
            # Check the distance between consecutive points
            distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
            if distance > 1:  # Adjust the threshold as needed
                plt.text(x, y, str(i), color='red')
                prev_x, prev_y = x, y
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Inverse Kinematics Visualization')
        plt.grid(True)
        plt.show()
        # Adjust the plot limits to show a larger area
        # saves the image in the given directory. Change it accordingly.
        #plt.savefig('/Users/mehradhq/Downloads/Robotics/img-end.png')
    else:
        print("the final joint variables are: ", result[0])
        print("the final position of the end-effector of the index finger is: ", result[1])
    
    initial_joint_angles = result[0]
  