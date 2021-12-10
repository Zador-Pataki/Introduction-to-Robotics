import numpy as np
from calculateFK import calculateFK
from distPointToBox import distPointToBox
from loadmap import loadmap
from copy import deepcopy
from pdb import set_trace


def velocityJacobian(joint, points_on_link, q): #
    """

    RETURN
    - Jvk: list of velocity Jacobian at position defined by joint and and points_on_link in qCurr
  
    Calculate the Jacobian of a particular joint of the  robot in a given configuration
    :param q: 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
    :param joint: scalar in [0,6] representing which joint we care about
    :return: J - 6 x (joint-1) matrix representing the Jacobian

    """

    if joint <= 2:
        return np.array([])

    FK = calculateFK()
    jointPositions, T0 = FK.forward(q)

    Jvk = []

    for point in range(1,points_on_link+2):
      Jv2 = np.zeros((3, joint-1))
      for i in range(joint-1):
        Jv2[:, i] = np.cross(T0[range(3), 2, i], (jointPositions[joint-1, :].T - T0[range(3), 3, i]) * point/(points_on_link+1))
      Jvk.append(Jv2)

    return Jvk

def force_attractive(pkCurr, pkGoal): #DONE FOR NOW
    """
    RETURN
    - Fk_attractive: attractive force applied to point k responsible for moving arm towards goal config
    """
    kapa = 10
    #xi = 0.01
    xi = 0.1
    if ( np.sum( (pkCurr-pkGoal)**2 ) )**0.5 > kapa:
        Fk_attractive = - (pkCurr - pkGoal)/  (np.sum( (pkCurr-pkGoal)**2 ) )**0.5
    else:
        #print("Parabolic ")
        Fk_attractive = - xi * (pkCurr - pkGoal)
    
    
    return Fk_attractive

def force_repulsive(pkCurr, map): #
    """
    RETURN
    - Fk_attractive: repulsive force applied to point k responsible for moving arm away from obstacles
    """
    rho = 50
    eta = 100000
    obstacles = map[0]
    boundary  = map[1]

    Fk_repulsive = np.zeros((1,3))
    for obstacle in obstacles:
        obstacle = obstacle.reshape((1,-1))
        
        pkCurr = pkCurr.reshape((1,-1))
        
        dist, unit = distPointToBox(pkCurr, obstacle)

        #if(dist==0): 
            #print("A collision occured")
            
        if dist>rho:
          Fk_repulsive += 0
        else:
          Fk_repulsive += eta * ((1/dist) - (1/rho)) * (1/dist**2) * -unit
        
        #print("Unit: ", unit)
        
    return Fk_repulsive


def force(pkCurr, map, pkGoal):
    """
    RETURN
    - Fk: force at point defined by k and K in qCurr
    """
    attr = force_attractive(pkCurr, pkGoal) 
    rep =  force_repulsive(pkCurr, map)
    #print("Attractive Force:", attr)
    #print("Repulsive Force:", rep)
    Fk = attr + rep
    return Fk

def joint_efforts_individual(joint, qCurr, map, qGoal, points_on_link): #
    """
    RETURN
    - tau_k joint efforts resulting from force Fk
    """
    fk = calculateFK()
    pkCurr_j = fk.forward(qCurr)[0][joint]
    pkCurr_pj = fk.forward(qCurr)[0][joint-1]

    pkGoal_j = fk.forward(qGoal)[0][joint]
    pkGoal_pj = fk.forward(qGoal)[0][joint-1]

    pkCurr = []
    pkGoal = []
    for i in range(1, points_on_link+2):
      pkCurr.append(pkCurr_pj + (pkCurr_j-pkCurr_pj)*i/(points_on_link+1)) #TODO calculate the goal position of point k in workspace
      pkGoal.append(pkGoal_pj + (pkGoal_j-pkGoal_pj)*i/(points_on_link+1)) #TODO calculate the current position of point k in workspace

    Jvk = velocityJacobian(joint, points_on_link, qCurr)
    tau = np.zeros(6)
    
    tau_k = np.zeros((joint-1,1))
    for i, Jv in enumerate(Jvk):
      Fk = force(pkCurr[i], map, pkGoal[i])
      #print("PkCurr", pkCurr[i], "i:", i, "JVK", len(Jvk), "joint", joint, "Shape JVK", Jv.shape)
      if(np.isnan(pkCurr[i]).any()):
        #set_trace()
        break
      #print(Fk.T.shape)
      tau_k += np.matmul(Jv.T , Fk.reshape((3,1)))    

    #print(tau_k.shape)
    tau[:joint-1] = tau_k[:,0]
    return tau

def joint_efforts_combination(qCurr, map, qGoal):
    """
    IN CONFIG SPACE RETURNS THE COMBINATION OF ALL JOINT EFFORTS RESULTING FORM INDIVIDUAL FORCES

    RETURN:
    - tau: [numpy 1x6]
    """
    points_on_link =  0#TODO number of points we wish to apply force to along liks (not including joints)
    
    tau = np.zeros(6)
    
    for k in range(2,6):
        tau += joint_efforts_individual(k, qCurr, map, qGoal, points_on_link)
    return tau


def potentialFieldStep(qCurr, map, qGoal, alpha=1e-3):
    """
    This function plans a path through the map using a potential field planner
    :param qCurr:       current pose of the robot (1x6).
    :param map:         the map struct
    :param qGoal:       goal pose of the robot (1x6).
    :return:
            qNext - 1x6 vector of the robots next configuration
            isDone - a boolean that is true when the robot has reached the goal or is stuck. false otherwise

    """
    epsilon = 1e-1      #TODO isDone condition
    tau = joint_efforts_combination(qCurr, map, qGoal) 
    qNext = qCurr + alpha * tau/np.linalg.norm(tau) 
        
    #if(np.isnan(qNext).any()):
    #    return qCurr, True
    
    isDone = False
    print("Dist to Goal",np.sqrt(np.sum( (qNext-qGoal)**2)))
    if np.sqrt(np.sum( (qNext-qGoal)**2)) < epsilon:
        isDone = True
        qNext = qGoal
        
    return qNext, isDone


