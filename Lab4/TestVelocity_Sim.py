#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand

from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController
from calculateFK import Main
import utils 

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from IK_velocity import IK_velocity
from FK_velocity import FK_velocity

fk = Main()


dq = [.5,-0.5,-0.5, -0.1,1.2,0]
dq = [-0.1,-0.1,-0.1,-0.1,-0.1,-0.1]

q=[0,0,0,0,0,0]
v=[fk.L1+fk.L2+fk.L3+fk.L4+fk.L5,1,1]
omega=[0,1,0]
joint = 0
#dq = IK_velocity(q, v, omega, joint)
dq = [0.1,0,0,0,0,0]
q =  [0,0,0,0,0,0]

v_, omega_ = FK_velocity(q, dq, joint)

dq = [0,1,0,0,0,0]
q =  [0,0,0,0,0,0]
v_, omega_ = FK_velocity(q, dq, joint)

'''
print("dq: ", dq)
print("v_: ", v_)
print("omega ", omega_)
'''

dq = [0, 0.1, 0, 0, 0, 0]
q =  [0, 0, 0, 0, 0, 0]
v = [ 4.38150000e+01,  1.78859665e-15, -2.68289498e-15] 
joint=6
omega = [0.,  0.3, 0.2]
print("dq:" , IK_velocity(q, v, omega, joint))
dq = IK_velocity(q, v, omega, joint)

v_, omega_ = FK_velocity(q, dq, joint)
print(v_)
print(omega_)
'''

v_, omega_ = FK_velocity(q, dq, joint)

print("dq: ", dq)
print("v_: ", v_)
print("omega ", omega_)
'''

dq = [-0.1,-0.1,-0.1,-0.1,-0.1,-0.1]

if __name__=='__main__':

    lynx = ArmController()
    sleep(1) # wait for setup

    print("Returning to Start Position:")
    lynx.set_pos([0,0,0,0,0,0])
    sleep(5)

    print("Target velocity:")
    print(dq)

    joint = 5
    x=[]
    y=[]
    z=[]
    
    q0=[]
    q1=[]
    q2=[]
    q3=[]
    q4=[]
    
    ###### GENERAL TRAJECTORY #####
    '''
    dq = [0.1,0.1,0.1,0.1,0.1,0.1]
    for i in range(100):
        q, vel = lynx.get_state()   
        v_, omega_ = FK_velocity(q, dq, joint)
        print("Setting dq :", dq[:])
        print("omega FK", omega_)
        print("v FK", v_)
        lynx.set_vel(dq)
        sleep(0.01)
        T0i = fk.forward(q)
        positions = T0i[-1][:-1,-1]
        x.append(positions[0])
        y.append(positions[1])
        z.append(positions[2])
        q0.append(q[0])
        q1.append(q[1])
        q2.append(q[2])
        q3.append(q[3])
        q4.append(q[4])    
    
    '''
    ##### MOVE IN A STRAIGHT LINE #####
    '''
    #lynx.set_pos([0,0,-np.pi/2,0,0,0])
    lynx.set_pos([0,0,0,0,0,0])
    sleep(5)
    v=[25,0,0]
    omega=[0,0,0]
    
    dq = IK_velocity(q, v, omega, joint)
    v_, omega_ = FK_velocity(q, dq, joint)
    print("Setting dq :", dq[:])
    print("omega FK", omega_)
    print("v FK", v_)

    omega=[np.nan,np.nan,np.nan]
    for i in range(100):
        q, vel = lynx.get_state()   
        dq = IK_velocity(q, v, omega, joint)
        v_, omega_ = FK_velocity(q, dq, joint)
        print("Setting dq :", dq[:])
        print("omega FK", omega_)
        print("v FK", v_)
        lynx.set_vel(dq)
        sleep(0.01)
        T0i = fk.forward(q)
        positions = T0i[-1][:-1,-1]
        x.append(positions[0])
        y.append(positions[1])
        z.append(positions[2])
        q0.append(q[0])
        q1.append(q[1])
        q2.append(q[2])
        q3.append(q[3])
        q4.append(q[4])
    '''
    ### MOVE IN A CIRCLE ###
    
    lynx.set_pos([0,-np.pi/4,np.pi/4,0,0,0])
    lynx.set_pos([0,0,0,0,0,0])
    sleep(5)
    q, vel = lynx.get_state()   
    T0i = fk.forward(q)
    position = T0i[-1][:-1,-1]
    
    alpha = 30
    center = position + alpha * np.array([0,0,1])
    ang_vel = np.array([0,0.5,0])
    
    omega = [np.nan, np.nan, np.nan]
    #omega = [0,0,0]
    
    for i in range(150):
        v = np.cross(ang_vel, position-center)
        dq = IK_velocity(q, v, omega, joint)
        v_, omega_ = FK_velocity(q, dq, joint)
        print("Setting dq :", dq[:])
        print("omega FK", omega_)
        print("v FK", v_)
        lynx.set_vel(dq)
        sleep(0.1)
        q, vel = lynx.get_state()
        T0i = fk.forward(q)
        position = T0i[-1][:-1,-1]
        x.append(position[0])
        y.append(position[1])
        z.append(position[2])
        q0.append(q[0])
        q1.append(q[1])
        q2.append(q[2])
        q3.append(q[3])
        q4.append(q[4])
    
    
    #lynx.set_vel(dq)
    #sleep(2)
    
    lynx.set_vel([0,0,0,0,0,0])

    pos, vel = lynx.get_state()

    print("Result:")
    print(pos)

    lynx.stop()
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x,y,z)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    #plt.savefig("trajectory.png")
    
    
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(q0, label='joint0')
    ax1.plot(q1, label='joint1')
    ax1.plot(q2, label='joint2')
    ax1.plot(q3, label='joint3')
    ax1.plot(q4, label='joint4')
    plt.legend()
    ax1.set_xlabel("Time")
    ax1.set_ylabel("Theta")
    

    plt.show()
    