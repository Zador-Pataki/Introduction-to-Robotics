#!/usr/bin/python2
import sys
from copy import deepcopy
from time import sleep
import numpy as np
from sys import path
from os import getcwd

path.append(getcwd() + "/../Core")
from arm_controller import ArmController
from calculateIK import Main
from calculateFK import Main as FK
# Uncomment the T0e for each target in the simulation, or enter your own

fk=FK()

#target 0
#T0e = np.asarray([[0,0,1,34+34+187.325],[0,-1,0,0],[1,0,0,76.2+146.05],[0,0,0,1]])
#T0e = np.asarray([[0,1,0,0],[0,0,1,34+34+187.325],[1,0,0,76.2+146.05],[0,0,0,1]])
# Target 1
T0e = np.array([[   0.019,    0.969,    0.245,   47.046],[   0.917,   -0.115,    0.382,   73.269],[   0.398 ,   0.217,   -0.891,  100.547],[   0.,       0. ,      0.,       1.]])

# Target 2
T0e = np.array([[  -0.993,   -0.,       0.119,  -96.936],[   0.,      -1.,      -0.,       0.   ],[   0.119,    0.,       0.993,  401.229],[   0. ,      0.  ,     0.  ,     1.   ]])

# Target 3
T0e = np.array([ [-0.3409003, -0.1074855,  0.9339346, 282.96],[0.7842780, -0.5802868,  0.2194888, -48.302],[0.5183581,  0.8072881,  0.2821184, 235.071 ], [0,0,0,1]])

# Target 4
#T0e = np.array([[  0.5054096, -0.8370580, -0.2095115, -45],[-0.0305796,  0.2252773, -0.9738147,-300],[0.8623375,  0.4985821,  0.0882604, 63 ],[0,0,0,1]])

# Please do not modify anything below this
if __name__=='__main__':
    main = Main()
    [q, isPos] = main.inverse(deepcopy(T0e))
    '''
    print("Target T0e: ", T0e)
    for i in range(q.shape[0]):
        print("joint values: ", q[i])
        jointTranslations, T0e1 = fk.forward(q[i,:])
        print("FK_" + str(i), T0e1)
        #print("Joint Translations: ", jointTranslations)
    '''
    np.set_printoptions(precision=6, suppress=True)
    cont = ArmController()
    sleep(1)
    print("Setup complete")
    if q.size != 0:
        cont.set_state(np.append(np.ravel(q)[range(5)],8))
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
    print('Simulation T0e =')
    print(pose[-1])
    print('Target T0e =')
    print(np.around(T0e, decimals=3))
    print('isPos =')
    print(isPos)
    print('q =')
    print(q)
    sleep(10)
    cont.stop()
    