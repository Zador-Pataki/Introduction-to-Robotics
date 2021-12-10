#!/usr/bin/python2
import sys
from time import sleep
import numpy as np
from sys import path
from os import getcwd
from numpy import pi

path.append(getcwd() + "/../Core")
from arm_controller import ArmController
from calculateFK import Main


# Modify q as configuration that running in the Gazebo simulation

q= [ 0.999995, -1.19755,   0.86032,   1.43694,   0.49947,   0.0      ]
# Please do not modify anything below this
if __name__=='__main__':
    main = Main()
    [jointPostions, T0e] = main.forward(q)
    np.set_printoptions(precision=6, suppress=True)
    cont = ArmController()
    sleep(1)
    print("Setup complete")
    
    cont.set_state(q)
    sleep(3)
        
    print('')
    print('The joint values are',cont.get_state()[0])
    print('')
    print('The joint velocities are', cont.get_state()[1])
    pose = cont.get_poses()
    poses = np.zeros((6, 3))
    for i in range (6):
        poses[i,0] = pose[i][0,3]
        poses[i,1] = pose[i][1,3]
        poses[i,2] = pose[i][2,3]
    print('')
    print('Simulation Joint Positions = ')
    print(poses)
    print('Predicted Joint Positions = ')
    print(np.around(jointPostions, decimals=3))
    print('Simulation T0e =')
    print(pose[-1])
    print('Predicted =')
    print(np.around(T0e, decimals=3))
    sleep(10)
    cont.stop()
