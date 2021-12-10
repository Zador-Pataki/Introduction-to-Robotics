import numpy as np
from numpy import matmul
from calculateFK import Main
import utils 

fk = Main()




def IK_velocity (q, v, omega, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any element is Nan, then that velocity can be
                  anything
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    dq - 1 x 6 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error.

    """
    d1 = 76.2                      # Distance between joint 0 and joint 1
    a2 = 146.05                    # Distance between joint 1 and joint 2
    a3 = 187.325                   # Distance between joint 2 and joint 3
    d4 = 34                        # Distance between joint 3 and joint 4
    d5 = 34                        # Distance between joint 3 and joint 5
    lg = 0                         # Distance between joint 5 and end effector (gripper length)

    dq = np.array([0, 0, 0, 0, 0, 0])

    #print(q.shape)
    J = utils.jacobian(q,joint)
    vel = np.concatenate((v,omega), axis=0)
    
    #print("Determinant of J", np.linalg.det(J))
    
    if np.isnan(vel[3]) and np.isnan(vel[4]) and np.isnan(vel[5]):
      J=J[:3, :]
    
    print("Jshape", J.shape)
    print("J: ", J)
    #Calculating Pseudo inverse
    try:
        if J.shape[0]<J.shape[1]:
            pseudo_inv = matmul(J.T, np.linalg.inv(matmul(J,J.T)))
        else:
            pseudo_inv = matmul(np.linalg.inv(matmul(J.T,J)) ,J.T)
    except:
        if J.shape[0]<J.shape[1]:
            pseudo_inv = matmul(J.T, np.linalg.inv(matmul(J,J.T) + 0.1*np.eye(J.shape[0])))
        else:
            pseudo_inv = matmul(np.linalg.inv(matmul(J.T,J) + 0.1*np.eye(J.shape[1])),J.T)
        
    if np.isnan(vel[3]) and np.isnan(vel[4]) and np.isnan(vel[5]):
      dq = np.dot(pseudo_inv , vel[:3])
      #dq,res,r,s = np.linalg.lstsq(J,vel[:3])      
    else:
      vel[np.isnan(vel)]=0
      dq = np.dot(pseudo_inv , vel)
      #dq,res,r,s = np.linalg.lstsq(J,vel)
      
    result = np.zeros((6))
    result[:dq.shape[0]] = dq

    return result