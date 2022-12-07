#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
 
    w_1 = [0,0,1]
    w_2 = [0,1,0]
    w_3 = [0,1,0]
    w_4 = [0,1,0]
    w_5 = [1,0,0]
    w_6 = [0,1,0]

    q_1 = [-150,150,10]
    q_2 = [-150, 270, 162]
    q_3 = [94,270,162]
    q_4 = [307,177,162]
    q_5 = [307,260,162]
    q_6 = [390,260,162]

    v_1 = -np.cross(w_1,q_1)
    v_2 = -np.cross(w_2,q_2)
    v_3 = -np.cross(w_3,q_3)
    v_4 = -np.cross(w_4,q_4)
    v_5 = -np.cross(w_5,q_5)
    v_6 = -np.cross(w_6,q_6)

    S_1 = np.array([[0,-w_1[2], w_1[1], v_1[0]], [w_1[2], 0, -w_1[0], v_1[1]], [-w_1[1], w_1[0], 0, v_1[2]],[0,0,0,0]])
    S_2 = np.array([[0,-w_2[2], w_2[1], v_2[0]], [w_2[2], 0, -w_2[0], v_2[1]], [-w_2[1], w_2[0], 0, v_2[2]],[0,0,0,0]])
    S_3 = np.array([[0,-w_3[2], w_3[1], v_3[0]], [w_3[2], 0, -w_3[0], v_3[1]], [-w_3[1], w_3[0], 0, v_3[2]],[0,0,0,0]])
    S_4 = np.array([[0,-w_4[2], w_4[1], v_4[0]], [w_4[2], 0, -w_4[0], v_4[1]], [-w_4[1], w_4[0], 0, v_4[2]],[0,0,0,0]])
    S_5 = np.array([[0,-w_5[2], w_5[1], v_5[0]], [w_5[2], 0, -w_5[0], v_5[1]], [-w_5[1], w_5[0], 0, v_5[2]],[0,0,0,0]])
    S_6 = np.array([[0,-w_6[2], w_6[1], v_6[0]], [w_6[2], 0, -w_6[0], v_6[1]], [-w_6[1], w_6[0], 0, v_6[2]],[0,0,0,0]])
    
    
    S = [S_1,S_2, S_3, S_4,S_5,S_6]
    



    M = [[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]]




    
    return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    # Initialize the return_value
    return_value = [None, None, None, None, None, None]

    
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    t_1 = np.dot(expm(S[0]*theta[0]),expm(S[1]*theta[1]))
    t_2 = np.dot(t_1, expm(S[2]*theta[2]))
    t_3 = np.dot(t_2, expm(S[3]*theta[3]))
    t_4 = np.dot(t_3,expm(S[4]*theta[4]))

    T_1 = np.dot(t_4,expm(S[5]*theta[5]))
    T = T_1*M

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    
    theta1 = 0.0
    theta2 = 0.0
    theta3 = 0.0
    theta4 = 0.0
    theta5 = 0.0
    theta6 = 0.0
    
    L = 53.5
    L1 = 152
    L2 = 120
    L3 = 244
    L5 = 213
    L7 = 83
    L8 = 82
    L9 = 53.5
    L10 = 59

    xb = xWgrip + 150
    yb = yWgrip - 150
    zb = zWgrip - 10 

    theta_yaw = np.radians(yaw_WgripDegree)

    xcen = xb - L*np.cos(np.radians(yaw_WgripDegree))
    ycen = yb - L*np.sin(np.radians(yaw_WgripDegree))
    zcen = zb
    
    d1 = L2 - 10;
    r = np.sqrt(xcen**2+ycen**2)
    
    theta1 = np.arctan2(ycen,xcen)-np.arctan2(d1,np.sqrt(r**2-d1**2))
    theta6 = PI/2 - theta_yaw + theta1
    
    x3end = xcen + d1*np.sin(theta1) - L7*np.cos(theta1)
    y3end = ycen - d1*np.cos(theta1) - L7*np.sin(theta1)
    z3end = L10 + L8 + zcen

    Lm = np.sqrt((z3end-L1)**2+x3end**2+y3end**2)
    
    theta3 = PI - np.arccos((L3**2+L5**2-Lm**2)/(2*L3*L5))
    
    theta2 = -np.arccos((L3**2+Lm**2-L5**2)/(2*L3*Lm))-np.arcsin((z3end-L1)/Lm)
    
    theta4 = -theta3 - theta2
    theta5 = -PI/2

    

    
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


