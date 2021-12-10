"""
MATLAB version AUTHOR:
    Dr. Cynthia Sung (crsung@seas.upenn.edu)
    Modified by Gedaliah Knizhnik (knizhnik@seas.upenn.edu) 08/28/19
Python version transformed AUTHOR:
    Zichen Lao (lao0910@seas.upenn.edu) 06/05/20
"""
import numpy as np
from numpy import sin, cos, pi, matmul

class Main():
    def __init__(self):
        self.L1 = 76.2    # distance between joint 0 and joint 1
        self.L2 = 146.05  # distance between joint 1 and joint 2
        self.L3 = 187.325 # distance between joint 2 and joint 3
        self.L4 = 34      # distance between joint 3 and joint 4
        self.L5 = 34      # distance between joint 4 and center of gripper

        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def returnTransform(self, a,alpha,d,theta):
        T = np.zeros((4,4))
        T[0,0] = cos(theta)
        T[0,1] = -cos(alpha)*sin(theta)
        T[0,2] = sin(theta)*sin(alpha)
        T[0,3] = cos(theta) * a
        T[1,0] = sin(theta)
        T[1,1] = cos(theta) * cos(alpha)
        T[1,2] = -cos(theta) * sin(alpha)
        T[1,3] = a * sin(theta)
        T[2,1] = sin(alpha)
        T[2,2] = cos(alpha)
        T[2,3] = d
        T[3,3] = 1
        
        return T
    
    def forward(self, q):
        T01= self.returnTransform(0,-pi/2, self.L1, q[0])
        T12= self.returnTransform(self.L2,0, 0, q[1]-(pi/2))
        T23= self.returnTransform(self.L3,0, 0, q[2]+(pi/2))
        T34 = np.asarray([[sin(q[3]), 0, cos(q[3]), self.L4 * cos(q[3])],
                           [-cos(q[3]), 0, sin(q[3]),self.L4* sin(q[3])],
                           [0, -1, 0 , 0],
                           [0 ,0, 0,1]])    
        

        T45 = self.returnTransform(0 ,0, self.L5, q[4])
        
        Tensors = [T01, T12, T23, T34, T45]

        d = np.zeros((4, 1)) 
        d[3,0] = 1

        jointPositions = np.zeros((6,3))
        T_intermediate = np.eye(4)

        T0i = []
        T0i.append(np.zeros((4,4)))
        T0i[0][:-1, :-1] = np.identity(3)
        for i in range(5):
            T_i = Tensors[i]
            if i == 0:
                T_intermediate = T_i
            else:
                T_intermediate = matmul(T_intermediate, T_i)
            T0i.append(T_intermediate)

            jointPositions[i+1, :] = matmul(T_intermediate, d)[:3,0]

        #print(T0i)        

        return T0i