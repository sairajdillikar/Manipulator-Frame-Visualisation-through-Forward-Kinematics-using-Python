import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# All the joint values are considered in degrees (currently at Pose A)
theta1 = (np.pi/180) * 0
theta2 = (np.pi/180) * 0
theta3 = (np.pi/180) * -90
theta4 = (np.pi/180) * 0
theta5 = (np.pi/180) * 0
theta6 = (np.pi/180) * 0

# Define DH parameters of the robot
# alpha, a, d, theta

# DH-Parameter referring the Kinova Gen-3 6 DoF from Base to Tip
dhparams = [[np.pi, 0, 0, 0],
            
            [np.pi/2, 0, -(156.43 + 128.38 + 100*np.pi), theta1],
            [np.pi, 410, -5.38, theta2-np.pi/2],
            [np.pi/2, 0, -6.38, theta3-np.pi/2],
            [np.pi/2, 0, -(208.43+105.93), theta4+np.pi],
            [np.pi/2, 0, 0, theta5+np.pi],
            [np.pi, 0, -(105.93+61.53), theta6+np.pi],
            ]

dof = len(dhparams)

# Define joint angles
theta = [0, 0, 0, 0, 0, 0, 0, 0]

# Define DH transform function
def dh_transform(alpha, a, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # Classic DH Homogenous Matrix
    C = np.array([[ct, -st*ca, st*sa, a*ct],
                  [st, ct*ca, -ct*sa, a*st],
                  [0, sa, ca, d],
                  [0, 0, 0, 1]])
    
    # Modified DH Homogenous Matrix
    M = np.array([[ct, -st, 0, a],
                  [st*ca, ct*ca, -sa, -d*sa],
                  [st*sa, ct*sa, ca, d*ca],
                  [0, 0, 0, 1]])
                  
    return C

# Define function to plot frame
def plot_frame(T, scale):
    origin = T[:3, 3]
    x_axis = origin + scale * T[:3, 0]
    y_axis = origin + scale * T[:3, 1]
    z_axis = origin + scale * T[:3, 2]
    plt.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], 'r')
    plt.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], 'g')
    plt.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], 'b')

# Plots the frames for given Classic DH-Table
def vis_frames(dhparams, dof):
    for i in range(dof):
        alpha, a_i, d_i, theta_i = dhparams[i]
        T_i = dh_transform(alpha, a_i, d_i, theta_i)
        if i == 0:
            T_base_i = T_i
        else:
            T_base_i = np.dot(T_base_i, T_i)            
        plot_frame(T_base_i, 50)

# Visualize the robot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-1500, 1500)
ax.set_ylim3d(-1500, 1500)
ax.set_zlim3d(-1500, 1500)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_box_aspect((2,2,2))

# To plot the World frame
plot_frame(np.eye(4), 200)

vis_frames(dhparams, dof)

plt.show()

