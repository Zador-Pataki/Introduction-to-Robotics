"""
dynamic_tracking.py: contains all relevant functions for tracking and 
picking up dynamic blocks
"""

#Import all relevant functions
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from copy import deepcopy
import sys
from os import getcwd
sys.path.append(getcwd() + "/../Core")
from calculateIK import Main as IKMain
from calculateFK import Main as FKMain
import time

IK = IKMain()
FK = FKMain() 

def our_dynamic_blocks(color,lynx):
    [name, pose, twist] = lynx.get_object_state()
    our_blocks_name = []

    for i, pose_i in enumerate(pose):
        x_i, y_i, z_i = pose_i[0, -1], pose_i[1, -1], pose_i[2, -1]
        r_i = ( x_i**2 + y_i**2 )*0.5
        if r_i < 105 and z_i > 0 and z_i < 70:
            our_blocks_name.append(name[i])
    return our_blocks_name


def generate_object_domain(pose):
    x_0 = pose[0, -1]
    y_0 = pose[1, -1]
    z_0 = pose[2, -1]
    r_0 = ( x_0**2 + y_0**2 )**0.5
    theta_0 = np.arctan2(y_0, x_0)
    theta_domain = np.linspace(theta_0, theta_0 + 2*np.pi, 100)
    
    feasible = np.logical_and((r_0*np.cos(theta_domain))**2 + (r_0*np.sin(theta_domain))**2 < 100**2, 
                              (r_0*np.cos(theta_domain)-200)**2 + (r_0*np.sin(theta_domain)-200)**2 + z_0**2<331.25**2)
    
    return theta_domain, feasible


def feasibility_domain(dynamic_blocks): 
    [name, pose, twist] = dynamic_blocks

    feasibility_domain_ = np.zeros((len(name), 2))  # col: (object_1:object_n)  row: (lb,ub)

    for i in range(len(name)):
   
        theta_domain_i, feasible_i = generate_object_domain(pose[i])

        if True not in feasible_i:
            print('EXCEPTION: NO FEASIBLE LOCATION FOUND FOR BLOCK', i, '          SUGGESTION: INCREASE SIZE OF GRID SEARCH')
            lb_i, ub_i = 0, 0 #QUICK SOL: LOWER BOUND IS UPPER BOUND ->>> FEASIBLE DOMAIN DOES NOT EXIST IN CONTINOUS TIME

            for j in range(theta_domain_i.shape[0]):
                if feasible_i[j-1] and not feasible_i[j]: ub_i = theta_domain_i[j-1]
                if feasible_i[j] and not feasible_i[j-1]: lb_i = theta_domain_i[j]

        try: feasibility_domain_[i, 0] = lb_i
        except: feasibility_domain_[i, 0] = theta_domain_i[0] #IF NO LOWEBOUND FOUND, LOWERBOUND OF DOMAIN IS LOWERBOUND
            
        try: feasibility_domain_[i, 1] = ub_i
        except: feasibility_domain_[i, 1] = theta_domain_i[0] #IF NO UPPERBOUND FOUND, UPPERBOUND OF DOMAIN IS UPPERBOUND
        
        if feasibility_domain_[i, 0] > 2*np.pi:
            feasibility_domain_[i,0] -= 2*np.pi
        if feasibility_domain_[i, 1] > 2*np.pi:
            feasibility_domain_[i,1] -= 2*np.pi

    return feasibility_domain_

def which_dynamic_block(dynamic_blocks, t_robot):
    """
    - dynamic_blocks: [name, pose, twist]
    """
    f_domain = feasibility_domain(dynamic_blocks)
    
    [name, pose, twist] = dynamic_blocks
    
    T_till_feasibility = []
    T_in_feasibility   = []
    
    for i in range(len(name)):
        x_0 = pose[i][0, -1]
        y_0 = pose[i][1, -1]
        theta_0 = np.arctan2(y_0, x_0)
        theta_dot = twist[i][-1]
    
        theta_predicted = predict_theta_block(theta_0, theta_dot, t_robot)
        t_till_feasibility = time_till_feasibility(theta_dot, theta_predicted, f_domain[i, 1], f_domain[i, 0])
        t_in_feasibility = time_in_feasibility(theta_dot, theta_predicted, f_domain[i, 1], f_domain[i, 0])
        T_till_feasibility.append(t_till_feasibility)
        T_in_feasibility.append(t_in_feasibility)
    
    idx = np.argmin(T_till_feasibility)
    name_dynam = name[idx] 
    return name_dynam, T_in_feasibility[idx], T_till_feasibility[idx]

def predict_theta_block(theta_0, theta_dot, t_robot):
    """
    PREDICTS THE POSE OF A BLOCK AFTER THE AMOUNT OF TIME IT TAKES FOR ROBOT TO PICK UP BLOCK
    """
    
    theta_predicted = theta_0 + theta_dot * t_robot
    return theta_predicted


def time_till_feasibility(theta_dot, theta_predicted, ub, lb):
    """
    TIME IT TAKES FOR BLOCK TO REACH FEASIBILITY DOMAIN
    """
    if ub < lb:  
        if ((theta_predicted > 0) and (theta_predicted < ub)) or ((theta_predicted > lb) and (theta_predicted < 2*np.pi)):
            time_till_feasibility_ = 0
        else:
            time_till_feasibility_ = (lb-theta_predicted) / theta_dot
    else:
        if theta_predicted > lb and theta_predicted < ub:
            time_till_feasibility_ = 0
        elif theta_predicted > ub:
            time_till_feasibility_ = (2*np.pi - theta_predicted + lb) / theta_dot
        else:
            time_till_feasibility_ = (lb - theta_predicted) / theta_dot
    
    return time_till_feasibility_


def time_in_feasibility(theta_dot, theta_predicted, ub, lb):
    """
    TIME BLOCK SPENDS IN FEASIBILITY DOMAIN IN CURRENT ROTATION CYCLE
    """
    if ub < lb: 
        if ((theta_predicted > 0) and (theta_predicted < ub)) or ((theta_predicted > lb) and (theta_predicted < 2*np.pi)):
            if theta_predicted > lb:
                feasible_domain_size = 2*np.pi + ub - theta_predicted
            else:
                feasible_domain_size = ub - theta_predicted
        else: 
            feasible_domain_size = 2*np.pi - lb + ub

    else: 
        if theta_predicted > lb and theta_predicted < ub:
            feasible_domain_size = ub - theta_predicted
        else: 
            feasible_domain_size = ub - lb
        
    time_in_feasibility = feasible_domain_size / theta_dot # assumption theta_t - theta_0 = theta_dot * t
    return time_in_feasibility



def getTransformations(T0e):
    T = []
    T.append(T0e)
    for i in range(3):
        T0e_new = T0e.copy()
        T0e_new[:,0] = T0e[:,1]
        T0e_new[:,1] = -T0e[:,0]
        T.append(T0e_new)
        T0e = T0e_new
    return T

def CanPick(ef_pose, box_pose, zdist):
    
    box_pose[2] += zdist
    distance_threshold = 30
    if(np.linalg.norm(ef_pose-box_pose)<distance_threshold):
        print("Allowed to Pick")
        return True
    else:
        return False
    
def stateToset(qt, qc):
    
    Qt = deepcopy(qt)
    Qc = deepcopy(qc)
    
    Qt[1] = 0 
    Qc[1] = 0
    
    
    Qt[4] = 0 
    Qc[4] = 0
    
    print("Distance: ", np.linalg.norm(Qt-Qc))

    if(np.linalg.norm(Qt-Qc)> 0.1):
        return Qt
    else:
        return qt 
    
def stateStep(qt, qc):
    if(np.linalg.norm(qc-qt)>0.1):
        qn = qc + (qt - qc)/np.linalg.norm(qc-qt)
    else:
        qn = qt 

    return qn

def grasp(q, pose, twist, T0e_target,lynx, block_name):
    
    grasped = False
    q[5] = -5
    lynx.command(q)
    sleep(0.1)
    
    T0e_target[2,3] += 30
    q, isPos = IK.inverse(T0e_target.copy())
    q = q[0,:]
    print("Last Solution", isPos)
    q[5] =  -5
    q[1] -= 0.3
    lynx.command(q)
    sleep(0.1)
    
    return True
    
    [names, pose, twist] = lynx.get_object_state()
    for i, name in enumerate(names):
            if name == block_name:
                print("HEIGHT OF BLOCK: ", pose[i][2,3])
                if(pose[i][2,3]>61):
                    grasped = True
                    break
                
    return grasped

def future_transform(T, omega, t):
    
    x , y = T[0,3],T[1,3]
    theta = np.arctan2(y,x)
    r = (x**2 + y**2)**0.5
    
    delta_theta = omega*t
    thetaf = theta + delta_theta
    xf =  r*np.cos(thetaf)
    yf =  r*np.sin(thetaf)
    R = T[:3,:3]
    R_rotate = np.zeros((3,3))
    R_rotate[0,0] = R_rotate[1,1] = np.cos(delta_theta)
    R_rotate[0,1] = -np.sin(delta_theta)
    R_rotate[1,0] = np.sin(delta_theta)

    T_new = T.copy()
    T_new[:3,:3] = np.matmul(R_rotate,R)
    T_new[0,3] = xf
    T_new[1,3] = yf
    
    return T_new


def picking_domain(theta_lb, theta_ub, pose, radius, color):
    x,y = pose[0,3], pose[1,3]
    
    
    if theta_lb<0:
        c1 = np.pi+theta_lb
    else:
        c1 = -np.pi + theta_lb
        
    if theta_ub<0:
        c2 = np.pi + theta_ub
    else:
        c2 = -np.pi + theta_ub
        
    
    r = (x**2 + y**2)**0.5
    #print(r)
    if r>radius:
        theta = np.arctan2(y,x)
        #print(theta)
        if color=='blue' and (theta<theta_lb or theta>theta_ub):
            return False
        elif color=='red' and (theta<c2 or theta>c1):
            return False
        else:
            return True
    else: 
        return True


def which_dynamic_block(dynamic_blocks, theta_lb_p, theta_ub_p, theta_lb_o, theta_ub_o, radius, color):
    [name, pose, twist] = dynamic_blocks

    choice = None
    min_radius = 1000
    print(name)
    for i in range(len(name)):
        if picking_domain(theta_lb_p, theta_ub_p, pose[i], radius, color):
            r = (pose[i][0,3]**2+pose[i][1,3]**2)**0.5
            if  r < min_radius:
                choice = name[i]
                min_radius = r
    
    if choice==None:
        print("Entering Orange Domain")
        for i in range(len(name)):
            #print(picking_domain(theta_lb_o, theta_ub_o, pose[i], radius, color))
            #print(theta_lb_o,"  ", theta_ub_o)
            theta = np.arctan2(pose[i][1,3],pose[i][0,3])
            print("Angle: ", theta)
            print(theta<theta_lb_o or theta>theta_ub_o)
            print(theta<theta_lb_o-np.pi or theta>theta_ub_o-np.pi)
            if picking_domain(theta_lb_o, theta_ub_o, pose[i], radius, color):
                r = (pose[i][0,3]**2+pose[i][1,3]**2)**0.5
                #print("AFTER ENTERING PICKING DOMAIN: ", r)
                #print("Min Radius: ", min_radius)
                if  r < min_radius:
                    choice = name[i]
                    min_radius = r
    return choice
            
    
def dynamicAction(lynx, color, block_name):

    [q,qd] = lynx.get_state()
    TR_W = np.zeros((4,4))
    TR_W[:3,3] = np.array([200,200,0])
    if color == 'blue':    
        TR_W[0,0] = -1
        TR_W[1,1] = -1
        TR_W[2,2] =  1
    else:
        TR_W[0,0] =  1
        TR_W[1,1] =  1
        TR_W[2,2] =  1        
    
    zero_config = [0,0,0,0,0,15]
    flat_config = [0,np.pi/2,-np.pi/2,0,0,0]
    
    zdist =  60
    picking = False
    grasped = False

    tf = 0.5
    
    theta_lb = -np.pi/8
    theta_ub = 3*np.pi/8
    radius = 5
    
    while True:
        [names, pose, twist] = lynx.get_object_state()
        [qc,qd] = lynx.get_state()
        T0i, jointPositions = FK.forward(qc) #Current State
        for i, name in enumerate(names):
            if name == block_name:
                T_future = future_transform(pose[i], twist[i][-1], tf)
                T0e = np.matmul(TR_W, T_future)
                axes_orientation = np.matmul(T0e[:3,:3].T, np.array([0,0,-1]).reshape(-1,1))
                idx = np.ones(4, dtype=np.int)
                idx[np.argmax(np.abs(axes_orientation))] = 0
                M = T0e[:3,idx]
                T0e_target = np.zeros((4,4))
                T0e_target[2,2]  = -1
                T0e_target[:3,0] = np.cross(M[:,1],T0e_target[:3,2])
                T0e_target[:3,1] = M[:,1]
                T0e_target[3,3]  = 1
                T0e_target[:3,3] = T0e[:3,3]
                
                         
                ef_pose = T0i[-1][:3,3].copy()
                box_pose = T0e[:3,3].copy()
                
                if not picking:
                    picking = CanPick(T0i[-1][:3,3].copy(), T0e_target[:3,3].copy(), zdist)
                  
                if not picking:
                    T0e_target[2,3] += zdist
                else:
                    T0e_target[2,3] += 5 
                    
                T = getTransformations(T0e_target)
                T0e_target_final = None
                isPos = False
                for transform in T:
                    q, isPos = IK.inverse(transform.copy())
                    q = q[0,:]
                    if isPos:
                        T0e_target_final = transform
                        break

                if not isPos:
                    print("Not possible")
                    return False
                
                height_threshold = 68
                distance_threshold = 25
                if(ef_pose[2]<height_threshold) and isPos:  
                    grasped = grasp(q,ef_pose,twist[i],T0e_target_final.copy(),lynx,block_name)
                    sleep(1)
                    for i in range(5):
                        [q,qd] = lynx.get_state()
                    
                    if(q[5]< 10):
                        picking = False 
                        return False
                    else:
                        return True
                else:
                    q[5] = 30
                if isPos:
                    qn = q
                    #sprint(q)
                    lynx.command(qn)
                
                sleep(0.01)
                
