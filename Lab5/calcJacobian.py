import numpy as np
from calculateFK import calculateFK

def calcJacobian(q, joint):
    """
    Calculate the Jacobian of a particular joint of the  robot in a given configuration
    :param q: 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
    :param joint: scalar in [0,6] representing which joint we care about
    :return: J - 6 x (joint-1) matrix representing the Jacobian
    """

    if joint <= 1:
        return np.array([])
    FK = calculateFK()
    jointPositions, T0 = FK.forward(q)

    Jw1 = np.zeros((3, joint-1))

    for i in range(joint-1):
        Jw1[:, i] = T0[range(3), 2, i]

    Jv2 = np.zeros((3, joint-1))
    for i in range(joint-1):
        Jv2[:, i] = np.cross(T0[range(3), 2, i], jointPositions[joint-1, :].T - T0[range(3), 3, i])
    J = np.vstack((Jv2, Jw1))
    return J
