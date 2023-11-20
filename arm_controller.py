# Import numpy for matrix operations
import numpy as np

# Define the DH parameters for Ned 2 robot
# Link lengths in mm
a2 = 221
d5 = 235


from controller import Robot


robotarm = Robot()


joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper::left", "gripper::right"]

initpos = [ +2.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

movepos = [np.pi/3, -np.pi/6, np.pi/6, np.pi/3, -np.pi/6, np.pi/6, 0.0, 0.0]

robot_joints = []

vel = 3
for i in range(8):
    temp = robotarm.getDevice(joint_names[i])
    robot_joints.append(temp)
    robot_joints[i].setPosition(initpos[i])
    robot_joints[i].setVelocity(vel)
    

vel = 0.5
for i in range(8):
    temp = robotarm.getDevice(joint_names[i])
    robot_joints.append(temp)
    robot_joints[i].setPosition(movepos[i])
    robot_joints[i].setVelocity(vel)
    
    
# Define a function that solves the forward kinematics for given joint angle vector

def forward_kinematics(theta):
    
    # Joint angles in radian
    theta1 = theta[0]
    theta2 = theta[1]
    theta3 = theta[2]
    theta4 = theta[3]
    theta5 = theta[4]
    theta6 = theta[5]

    
    # Define the homogenous transformation matrices for each frame using DH parameters
    #-----------------------------T01-----------------------------
    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                    [np.sin(theta1),  np.cos(theta1), 0, 0],
                    [     0,                0,        1, 0],
                    [     0,                0,        0, 1]
                   ])
    #-------------------------------------------------------------

    #-----------------------------T12-----------------------------
    Rx_n90 = np.array([[1,  0, 0, 0],
                       [0,  0, 1, 0],
                       [0, -1, 0, 0],
                       [0,  0, 0, 1]
                   ])

    Rz_theta2 = np.array([[np.cos(theta2), -np.sin(theta2), 0, 0],
                         [np.sin(theta2),   np.cos(theta2), 0, 0],
                         [     0,                 0,        1, 0],
                         [     0,                 0,        0, 1]
                   ])

    T12 = Rx_n90 @ Rz_theta2

    #T12 = np.array([[ np.cos(theta2), -np.sin(theta2), 0, 0],
    #                [      0,                0,        1, 0],
    #                [-np.sin(theta2), -np.cos(theta2), 0, 0],
    #                [      0,                0,        0, 1]
    #               ])
    #-------------------------------------------------------------

    #-----------------------------T23-----------------------------
    T23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, a2],
                    [np.sin(theta3),  np.cos(theta3), 0,  0],
                    [     0,                0,        1,  0],
                    [     0,                0,        0,  1]
                   ])
    #-------------------------------------------------------------

    #-----------------------------T34-----------------------------
    Rz_theta4 = np.array([[np.cos(theta4), -np.sin(theta4), 0, 0],
                          [np.sin(theta4),  np.cos(theta4), 0, 0],
                          [     0,                0,        1, 0],
                          [     0,                0,        0, 1]
                   ])

    T34 = Rx_n90 @ Rz_theta4

    #T34 = np.array([[ np.cos(theta4), -np.sin(theta4), 0, 0],
    #                [      0,                0,        1, 0],
    #                [-np.sin(theta4), -np.cos(theta4), 0, 0],
    #                [      0,                0,        0, 1]
    #               ])
    #-------------------------------------------------------------

    #-----------------------------T45-----------------------------

    Rx_90 = np.array([[1,  0,  0, 0],
                      [0,  0, -1, 0],
                      [0,  1,  0, 0],
                      [0,  0,  0, 1]
                   ])

    Rz_theta5 = np.array([[np.cos(theta5), -np.sin(theta5), 0, 0],
                          [np.sin(theta5),  np.cos(theta5), 0, 0],
                          [     0,                0,        1, 0],
                          [     0,                0,        0, 1]
                   ])

    T45 = Rx_90 @ Rz_theta5
    T45[1][3] = d5
    #T45 = np.array([[ np.cos(theta5), -np.sin(theta5),  0, 0],
    #                [      0,                0,        -1, 0],
    #                [ np.sin(theta5),  np.cos(theta5),  0, 0],
    #                [      0,                0,         0, 1]
    #               ])
    #-------------------------------------------------------------

    #-----------------------------T56-----------------------------
    Rz_theta6 = np.array([[np.cos(theta6), -np.sin(theta6), 0, 0],
                          [np.sin(theta6),  np.cos(theta6), 0, 0],
                          [     0,                0,        1, 0],
                          [     0,                0,        0, 1]
                   ])

    T56 = Rx_n90 @ Rz_theta6

    #T56 = np.array([[ np.cos(theta6), -np.sin(theta6), 0, 0],
    #                [      0,                0,        1, 0],
    #                [-np.sin(theta6), -np.cos(theta6), 0, 0],
    #                [      0,                0,        0, 1]
    #               ])
    #-------------------------------------------------------------

    # Calculate the forward kinematics by multiplying the transformation matrices
    #-----------------------------T06-----------------------------
    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    #-------------------------------------------------------------
    
    # Extract the position and Orientation of the end-effector
    position = T06[:3,3]   # The first three data of the last column of the matrix 
    orientation = T06[:3,:3]  # The upper left 3x3 submatrix
    
    # Return all the important parameters
    return (T01, T12, T23, T34, T45, T56, T06, position, orientation)
    



# Testing..........................
theta_test = [np.pi/3, -np.pi/6, np.pi/6, np.pi/3, -np.pi/6, np.pi/6]
# Getting all the important parameters from the forward_kinematics function using theta_test as input
T01, T12, T23, T34, T45, T56, T06, position, orientation = forward_kinematics(theta_test)

# Printing all the parameters
print("T01  = \n" + str(T01) + "\n")
print("T12  = \n" + str(T12) + "\n")
print("T23  = \n" + str(T23) + "\n")
print("T34  = \n" + str(T34) + "\n")
print("T45  = \n" + str(T45) + "\n")
print("T56  = \n" + str(T56) + "\n")
print("T06  = \n" + str(T06) + "\n")
print("Position = \n" + str(position) + "\n")
print("Orientation = \n" + str(orientation))
