import numpy as np
import math
from numpy import cos
from numpy import sin

class Main():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 1 and joint 2
        self.a2 = 146.05                    # Distance between joint 2 and joint 3
        self.a3 = 187.325                   # Distance between joint 3 and joint 4
        self.d4 = 34.0                        # Distance between joint 4 and joint 5
        self.d5 = 34.0                       # Distance between joint 4 and end effector

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)
    
    def report_end_effector_workspace(self, T_e, end_effector_workspace=True): #first test otherwise no need to do rest
        t1, t2, t3 = T_e[0, -1], T_e[1, -1], T_e[2, -1]
        radius_end_effector = (t1**2 + t2**2 + (t3-self.d1)**2)**0.5
        radius_limit = self.a2 + self.a3 + self.d4 + self.d5

        if radius_end_effector > radius_limit:
          end_effector_workspace = False
        return end_effector_workspace

    def report_feasible(self, T_e, isPos = True): #returns whether or not orientation is feasable given that worksapce is true
  
        t1 = T_e[0, 3]
        t2 = T_e[1, 3]
        theta_1 = np.arctan2(t2, t1)

        Rz_1 = np.array([-np.sin(theta_1), np.cos(theta_1), 0])
        Rz_e = Rz_e = T_e[:-1, 2]

        print(Rz_1)
        print(Rz_e, '***')
        print(np.abs(np.dot(Rz_1, Rz_e)))
        if not np.abs(np.dot(Rz_1, Rz_e)) < 1e-5:
          isPos = False

        return isPos, Rz_1

    def feasible_transform_matrix(self, T_e, Rz_1): #only gets called if matrix is not feasable and end effector workspace is true
        Ry_e = T_e[:-1, 1]
        Rz_e = T_e[:-1, 2]

        Rz_e_f = ( Rz_e - np.dot(Rz_e, Rz_1) * Rz_1 ) / np.linalg.norm( Rz_e - np.dot(Rz_e, Rz_1) * Rz_1 )

        Ry_e_f = ( Ry_e - np.dot(Ry_e, Rz_e_f) * Rz_e_f ) / np.linalg.norm( Ry_e - np.dot(Ry_e, Rz_e_f) * Rz_e_f )

        Rx_e_f = np.cross(Ry_e_f, Rz_e_f)

        Rx_e_f = Rx_e_f[:, np.newaxis]
        Ry_e_f = Ry_e_f[:, np.newaxis]
        Rz_e_f = Rz_e_f[:, np.newaxis]

        R_e_f = np.concatenate((Rx_e_f, Ry_e_f, Rz_e_f), axis=1)
        
        print("Original T_E: " , T_e)
        T_e[:-1, :-1] = R_e_f
        print("Final T_E: ", T_e)
        
        return T_e

    
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

    def joint_angles(self, T0e, theta_1, theta_2, theta_3):
        T01 = self.returnTransform(0,-np.pi/2, self.d1, theta_1)
        T12 = self.returnTransform(self.a2, 0, 0, theta_2 - (np.pi/2))
        T23 = self.returnTransform(self.a3,0, 0, theta_3  + (np.pi/2))

        #print("T01", T01)
        #print("T12", T12)
        #print("T23", T23)

        T03 =  np.matmul(np.matmul(T01,T12),T23)
        R03 =  T03[:3,:3]
        R05 =  T0e[:3,:3]
        
        #Rotation of Wrist Joints
        R35 = np.matmul(R03.T,R05)
        
        #Re-normalizing Vectors
        R35[:,0] = R35[:,0] / np.linalg.norm(R35[:,0])  
        R35[:,1] = R35[:,1] / np.linalg.norm(R35[:,1])
        R35[:,2] = R35[:,2] / np.linalg.norm(R35[:,2])
        
        #print("R35", R35)

        #R34 = np.array([[sin(theta_4), 0, -cos(theta_4)], [-cos(theta_4) ,0, +sin(theta_4)], [0,  -1, 0]])
        #R45 = np.asrray([[cos(theta_5), cos(theta_5), 0], [sin(theta_5), -sin(theta_5), 0], [0, 0, 1]])
        
        #theta_4 = np.arccos(R35[0,2]) #Using cos(-theta) = cos(theta)
        #theta_5 = np.arccos(-R35[2,1]) + np.pi
        #theta_5 =0
        theta_4 = np.arctan2(R35[1,2],R35[0,2])
        theta_5 = np.arctan2(R35[2,0],R35[2,1]) + np.pi
        theta_5 = np.arctan2(-R35[2,0],-R35[2,1])
        #theta_5 = np.arcsin(-R35[2,0])
        print("DEBUG: ", R35[2,1], R35[0,2])
        
        '''
        if theta_1 > np.pi/2 or theta_1 < -np.pi/2:
            sign2 = -1
        else:
            sign2 = 1
        sign = np.sign(R05[0, 2])
            
        theta_z = sign * sign2 * np.arccos(R05[2, 2])
        theta_4 = theta_z - np.pi/2 -theta_3-theta_2
        
                
        R34 = [[sin(theta_4), 0, cos(theta_4)], [-cos(theta_4), 0, sin(theta_4)], [0, -1, 0]]
        print(R34)
        R45 = np.matmul(np.matmul(R03,R34).T, R05)
        theta_5 = np.arcsin(R45[1, 0])
        '''
        q = np.zeros((1,6))
        q[0,0] = theta_1
        q[0,1] = theta_2
        q[0,2] = theta_3
        q[0,3] = theta_4
        q[0,4] = theta_5
        return q

    def inverse(self, T0e):
        """
        INPUT:
        T - 4 x 4 homogeneous transformation matrix, representing
           the end effector frame expressed in the base (0) frame
           (position in mm)

        OUTPUT:
        q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad)
           which are required for the Lynx robot to reach the given
           transformation matrix T. Each row represents a single
           solution to the IK problem. If the transform is
           infeasible, q should be all zeros.
        isPos - a boolean set to true if the provided
             transformation T is achievable by the Lynx robot as given,
             ignoring joint limits
        """
        isPos = False
        if self.report_end_effector_workspace(T0e):
          print("End-effector is inside of end-effector workspace")
          isPos, Rz_1 = self.report_feasible(T0e)

          print(isPos)

          if not isPos:
            print("End-effector orrientation is not feasible")
            T0e = self.feasible_transform_matrix(T0e, Rz_1)
            print("End-effector matrix T_e redefined:\n", T0e)

        q = np.zeros((4, 6))
        # Your code starts from here
        
        #Calculating position of wrist center
        wrist_length = self.d4 + self.d5
        x, y, z = T0e[0,3]- wrist_length*T0e[0,2], T0e[1,3]- wrist_length*T0e[1,2], T0e[2,3]- wrist_length*T0e[2,2]
        
        print("Wrist Center x: %f, y: %f, z:%f " %(x,y,z))
        
        ##Check if wrist center is outside workspace
        if np.sqrt((z-self.d1)**2 + x**2 + y**2) > self.a2 + self.a3:
            print("Wrist Center Outside Joint Limits: (%d > %d)" %(np.sqrt((z-self.d1)**2 + x**2 + y**2) , self.a2 + self.a3))
            return q, isPos
        
        #Joint angle given wrist center co-ordinates
        theta_1 = np.arctan2(y, x) 
        theta_3a= -np.arccos(((x**2 + y**2) + (z-self.d1)**2 - self.a2**2 - self.a3**2 )/(2*self.a2*self.a3)) -np.pi/2
        theta_2a = np.pi/2 - np.arctan2((z-self.d1),(x**2+y**2)**0.5)+np.arctan2(self.a3 * np.sin(-theta_3a-np.pi/2), self.a2 + self.a3 * np.cos(-theta_3a -np.pi/2))

        theta_3b = -theta_3a + np.pi
        theta_2b = np.pi/2 - np.arctan2((z-self.d1),(x**2+y**2)**0.5)+np.arctan2(self.a3 * np.sin(-theta_3b-np.pi/2), self.a2 + self.a3 * np.cos(-theta_3b -np.pi/2))
        
        q0=self.joint_angles(T0e, theta_1, theta_2a, theta_3a)
        q1=self.joint_angles(T0e, theta_1, theta_2b, theta_3b)
        
        q2=self.joint_angles(T0e, theta_1 + np.pi, -theta_2a, theta_3b)        
        q3=self.joint_angles(T0e, theta_1 + np.pi, -theta_2b, theta_3a)
        
        qs = [q0, q1, q2, q3]
        q_cleaned =[]
        for i, Q in enumerate(qs):
            
            Q = Q%(2*np.pi)
            
            Q[Q>np.pi] =   -(2*np.pi -Q[Q>np.pi])
            Q[Q<-np.pi] =  -(-2*np.pi -Q[Q<-np.pi])
        
        
            idx = Q>self.upperLim
            if idx.any():
                continue
            
            idx = Q<self.lowerLim
            if idx.any():
                continue
            
            #Q[idx] = self.upperLim[idx]
            #print("Exceeding Upper limit: ", idx)
            #Q[idx] = self.lowerLim[idx]
            #print("Below Lower limit: ", idx)
            #print("Q: ", Q)
            q_cleaned.append(Q)
         
        if(len(q_cleaned)):    
            q = np.concatenate(tuple(q_cleaned), axis=0) 
        else:
            q = np.empty((1,6))
        print("qshape: ", q.shape)
        # Your code ends here
        return q, isPos
    
#Basic Test Cases

#Ik  = Main()
#T0e = np.asarray([[0,0,1,34+34+187.325],[0,-1,0,0],[1,0,0,76.2+146.05],[0,0,0,1]])
#T0e = np.asarray([[0,1,0,0],[0,0,1,34+34+187.325],[1,0,0,76.2+146.05],[0,0,0,1]])
#q, isPos = Ik.inverse(T0e)
#print(q)
#T0e = np.asarray([[0,0,1,34+68+187.325],[0,-1,0,0],[1,0,0,76.2+146.05],[0,0,0,1]])


#print("q:", q)
