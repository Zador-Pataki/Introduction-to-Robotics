#!/usr/bin/python2
import sys
from time import sleep
import numpy as np
from sys import path
from os import getcwd

path.append(getcwd() + "/../Core")
from arm_controller import ArmController

import numpy as np
from calculateFK import Main
from numpy import pi

### Q 2.4.1
print("2.4.1")
q=[0,0,0,0,0,0]
FK=Main()
jointPositions, T0e = FK.forward(q)
print("q: ", q)
print("Joint Positions: ", jointPositions)
print("T0e: ", T0e)

### Q 2.4.2
print("2.4.2")
q=[pi/4, 0, 0, 0, 0, 0]
jointPositions, T0e = FK.forward(q)
print("q: ", q)
print("Joint Positions: ", jointPositions)
print("T0e: ", T0e)

print("2.4.2")
q=[-pi/2, 0, pi/4, 0, pi/2, 0]
jointPositions, T0e = FK.forward(q)
print("q: ", q)
print("Joint Positions: ", jointPositions)
print("T0e: ", T0e)


### Q 2.4.4 BEGINNING
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

"""
Creating Arm controller to compare results with the simulation
"""
cont = ArmController()
sleep(1)
print("setup complete")

"""
Generating evenly spaced inputs within the joint limits for each joint
qi - resolution x 1 vector
"""
resolution = 2
q0 = np.linspace(-1.4, 1.4, resolution)
q1 = np.linspace(-1.2, 1.4, resolution)
q2 = np.linspace(-1.8, 1.7, resolution)
q3 = np.linspace(-1.9, 1.7, resolution)
q4 = np.linspace(-2.0, 1.5, resolution)
q5 = np.linspace(-15, 30, resolution)

"""
x, y, z: lists that, after running the following loop, will contain the outputs
         FK.forward() to all combinations of inputs specified above
x_cross, y_cross, z_ross: lists that, after running the following loop, 
         will contain the outputs FK.forward() to all combinations of inputs
         q1 to q2 specified above, at individual q0 values
"""
x = []
y = []
z = []

x_f=[]
y_f=[]
z_f=[]

translation_error= []
rotation_error = []

for i, a in enumerate(q0):
    x_cross = []
    y_cross = []
    z_cross = []
    for b in q1:
        for c in q2:
            for d in q3:
                for e in q4:
                    #print ([a, b, c, d, e,0])
                    jointPositions, _ = FK.forward([a, b, c, d, 0,0])
                    
                    out_i = jointPositions[5,:]

                    x.append(out_i[0])
                    y.append(out_i[1])
                    z.append(out_i[2])

                    x_cross.append(out_i[0])
                    y_cross.append(out_i[1])
                    z_cross.append(out_i[2])
                
                    
                    print("Setting New State")
                    
		    
                    while np.linalg.norm(np.asarray([a,b,c,d]) - cont.get_state()[0][:4])>1e-3:
                      print('print a', [a,b,c,d,0])
                      cont.set_state([a,b,c,d,0,0])
                      sleep(1)
                      print('print b', cont.get_state()[0][:5])
                      print("NORM:", np.linalg.norm(np.asarray([a,b,c,d]) - cont.get_state()[0][:4]))
                    pose = cont.get_poses()
                    poses = np.zeros((6, 3))
                    #cont.stop()
                    for i in range (6):
                        poses[i,0] = pose[i][0,3]
                        poses[i,1] = pose[i][1,3]
                        poses[i,2] = pose[i][2,3]
                    
                    """
                    Comparing Simulation and Predicted Results
                    """
                    print('The joint values are',cont.get_state()[0])
                    print("Ours, ",np.around(jointPositions, decimals=3))
                    print("Theirs ", poses)
                    print("Ours, ", np.around(_, decimals=3))
                    print("Theirs ", pose[-1])
                    compare = np.around(jointPositions, decimals=3) == poses
                    
                    #assert compare.all() == True
                    translation_error.append(np.linalg.norm(jointPositions - poses)/np.mean(np.absolute(jointPositions)))
                    
                    try:
                        assert np.linalg.norm(jointPositions - poses)/np.mean(np.absolute(jointPositions))<1e-2
                    except:
                        x_f.append(out_i[0]), y_f.append(out_i[1]), z_f.append(out_i[2])
                        print("failed Translation")
                        
                    #compare = np.around(_, decimals=3) == pose[-1]
                    #assert compare.all() == True
                    print(np.linalg.norm(_[:3,:3] - pose[-1][:3,:3])/np.mean(np.absolute(_[:3,:3])))
                    rotation_error.append(np.linalg.norm(_[:3,:3] - pose[-1][:3,:3])/np.mean(np.absolute(_[:3,:3])))
                    
                    try:
                        assert np.linalg.norm(_[:3,:3] - pose[-1][:3,:3])/np.mean(np.absolute(_[:3,:3]))<1e-2
                    except:
                        print("Failed Rotation")
                    
    """
    Plotting scatter plots of individual cross sections of the reachable 
    workspace at all q0 values specified 
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_cross,y_cross,z_cross)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.savefig("view_cross_section_" + str(i) + ".png")
    plt.show()   

"""
Plotting a scatter plot of the entire workspace
"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x,y,z)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.savefig("reachable_workspace.png")
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_f,y_f,z_f)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.savefig("reachable_failed_workspace.png")
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(rotation_error, c='r', label='Rotation Error')
ax.plot(translation_error, c='b', label='Translation Error')
ax.set_xlabel("Trials")
ax.set_ylabel("Error")

plt.savefig("errors.png")
plt.show()

### Q 2.4.4 END
