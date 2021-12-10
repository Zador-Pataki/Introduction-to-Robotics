#!/usr/bin/python2
import numpy as np 
from numpy import cos 
from numpy import sin
from numpy import pi
#from numpy import dot as matmul
from numpy import matmul
class Main():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable
        """
        # Lynx Dimensions in mm
        self.L1 = 76.2    # distance between joint 0 and joint 1
        self.L2 = 146.05  # distance between joint 1 and joint 2
        self.L3 = 187.325 # distance between joint 2 and joint 3
        self.L4 = 34      # distance between joint 3 and joint 4
        self.L5 = 34      # distance between joint 4 and center of gripper

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

### Q 2.3.3 BEGINNING
    def returnTransform(self, a,alpha,d,theta):
        """
        INPUT:
        a, alpha, d, theta - Scalar values of DH parameters
        OUTPUTS:
        T - 4 x 4 matrix; a homogenous transformation matrix according to the 
            DH convenction between the coresponding coordinate frames of two 
            consectutive links
        """
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
        """
        INPUT:
        q - list of joint inputs [q0,q1,q2,q3,q4,lg]
        OUTPUTS:
        jointPositions - 6 x 3 matrix, where each row represents one
                  joint along the robot. Each row contains the [x,y,z]
                  coordinates of the respective joint's center (mm). For
                  consistency, the first joint should be located at
                  [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  base (0) frame
        """
      
        '''  
        for i in range(len(q)):
            if q[i] > self.upperLim[0, i]: 
                print('q element ' + str(i) + ' exceeded joint limit')
                q[i] = self.upperLim[0, i]
            elif q[i] < self.lowerLim[0, i]: 
                print('q element ' + str(i) + ' exceeded joint limit')
                q[i] = self.lowerLim[0, i]
        
        print(q)
        '''
        T01= self.returnTransform(0,-pi/2, self.L1, q[0])
        T12= self.returnTransform(self.L2,0, 0, q[1]-(pi/2))
        T23= self.returnTransform(self.L3,0, 0, q[2]+(pi/2))



        """
        T34 - 4 x 4 homegenous transformation matrix between links 3 and 4 
              contructed through geometric intuition as opposed to following to
              DH convention in order to preserve the joint location
        """
        T34 = np.asarray([[sin(q[3]), 0, cos(q[3]), self.L4 * cos(q[3])],
                           [-cos(q[3]), 0, sin(q[3]),self.L4* sin(q[3])],
                           [0, -1, 0 , 0],
                           [0 ,0, 0,1]])    
        

        T45 = self.returnTransform(0 ,0, self.L5, q[4])
        
        Tensors = [T01, T12, T23, T34, T45]

        d = np.zeros((4, 1)) 
        d[3,0] = 1

        """
        d - 4 x 1 homogenous representation of the vector of the location of 
            the origin of the base from in the base frame
        T_intermediate - 4 x 4 matrix storing homogenous transformations at 
                         intermediate iterations between base frame and 
                         corresponding intermediate joint frames 
        """

        jointPositions = np.zeros((6,3))
        T_intermediate = np.eye(4)
       
        for i in range(5):
            T_i = Tensors[i]
            if i == 0:
                T_intermediate = T_i
            else:
                T_intermediate = matmul(T_intermediate, T_i)

            jointPositions[i+1, :] = matmul(T_intermediate, d)[:3,0]

        T0e = T_intermediate
        
        return jointPositions, T0e
### Q 2.3.3 END

q= [ 0.999995, -1.19755,   0.86032,   1.43694,   0.49947,   0.0      ]
fk = Main()
_,T0e= fk.forward(q)
print(T0e)