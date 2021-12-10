import numpy as np
from potentialFieldStep import potentialFieldStep
from loadmap import loadmap
from copy import deepcopy
from rrt import rrt
from rrt import isColliding
from time import sleep

def dummy_potentialFieldPath(map, qStart, qGoal,alpha=1e-4):
    qPath = qStart.reshape((-1,6))
    isDone = False
    local_minima = False
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))

    max_steps= 10000
    while not isDone:
        qNext, isDone = potentialFieldStep(qPath[-1,:], map, qGoal, alpha)
        qNext = qNext.reshape((1,-1))
        print("qNext", qNext)

        qNext[qNext > upperLim] = upperLim[qNext > upperLim] - 0.01
        qNext[qNext < lowerLim] = lowerLim[qNext < lowerLim] + 0.01

        q_check = qPath[-50:, :]
        diff = (q_check[0, :] - qNext)
        
        error = abs(np.linalg.norm(diff))
        #print("ERROR", error,  "  NUMBER OF STEPS: ", qPath.shape[0])
        if abs(error) > 1e-2 or qPath.shape[0]<50:
            local_minima = False
        else:
            #print("Found a local Minina")
            local_minima=True
            
        qPath = np.concatenate((qPath, qNext), axis=0)
        if local_minima:
            break
        
        if(qPath.shape[0]>max_steps):
            isDone=True
        
    return qPath, isDone

def potentialFieldPath(map, qStart, qGoal,alpha=1e-4):
    """
    Implement RRT algorithm in this file.
    :param map:        the map struct
    :param qStart:     start pose of the robot [numpy 1x6]
    :param qGoal:      goal pose of the robot [numpy 1x6]
    :return:           returns an [numpy Nx6] matrix, where each row consisits of the configuration of the Lynx at a point on the path. The first row is  start and the last
                       row is goal. If no path is found, PATH is a 0x6 matrix.
    """
    qPath = qStart.reshape((-1,6))
    
    if(isColliding(qStart, map) or isColliding(qGoal, map)):
        print("Start or End not Viable")
        return qStart

    
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))
    if( (qStart>upperLim).any() or (qStart<lowerLim).any() or (qGoal<lowerLim).any() or (qGoal>upperLim).any()):
        print("Outside joint limits")
        return qStart
    
    epsilon = 0.5
    isDone = False
    #print(qPath.shape)
    #print(qPath.reshape((-1,6))[-1,:].shape)
    #print("qpath" , qPath)
    #will need rrt.py 
    local_minima = False
    pot_from_rrt = False
    
    max_steps = 1000
    while not isDone:
        qNext, isDone = potentialFieldStep(qPath[-1,:], map, qGoal, alpha)
        qNext = qNext.reshape((1,-1))
        print("qNext", qNext)

        qNext[qNext > upperLim] = upperLim[qNext > upperLim] -0.01
        qNext[qNext < lowerLim] = lowerLim[qNext < lowerLim] +0.01

        q_check = qPath[-50:, :]
        diff = (q_check[0, :] - qNext)
        
        error = np.linalg.norm(diff)
        #print("ERROR", error)
        #if True:
        if abs(error) > 5*1e-3 or qPath.shape[0]<50:
            local_minima = False
        else:
            print("Found a local Minina")
            local_minima=True
        
        if local_minima and np.sqrt(np.sum((qNext-qGoal)**2)) > epsilon:
            print("STUCK IN A LOCAL MINIMA: Distance to goal: ", np.sqrt(np.sum((qNext-qGoal)**2)))
            
            sleep(1)
            qRRT = rrt(map, qPath[-1, :], qGoal) # Nx6 matri
            print("START: ", qPath[-1,:])
            print("END :", qGoal)
            #print(type(qRRT))
            #print(qRRT)
            RRT_size = qRRT.shape[0]
            print("RRT SIZE: ", RRT_size)
            print(int(RRT_size /10))
            for i in range(int(RRT_size /10)):
                qDummy = qRRT[i*10, :]
                qRRT_pot, pot_from_rrt = dummy_potentialFieldPath(map, qDummy, qGoal, alpha) 
                #print("THIS ONE IS ALSO A MINIMA", pot_from_rrt)
                if pot_from_rrt:
                    qPath = np.concatenate((qPath, qRRT[:i*10, :]), axis=0)
                    qPath = np.concatenate((qPath, qRRT_pot[:1000,:]), axis=0)
                    qPath = np.concatenate((qPath, qGoal.reshape((1,-1))), axis=0)
                    #print("SUCEEDED WITH RRT")
                    sleep(1)
                    return qPath

        else:
            qPath = np.concatenate((qPath, qNext), axis=0)
            #sleep(0.1)
        #print("Number of steps: ", qPath.shape[0])
        
        if(qPath.shape[0]>max_steps):
            #print(qGoal.shape)
            #print(qPath.shape)
            qPath = np.concatenate((qPath, qGoal.reshape((1,-1))), axis=0)
            break
        
        
    return qPath
