import numpy as np
from calculateFK import Main
from numpy import matmul
import utils
fk = Main()


def FK_velocity (q, dq, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration
    :param dq: 1 x 6 vector corresponding to the robot's current joint velocities
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    v     - The resulting linear velocity in the world frame
    omega - The resulting angular velocity in the world frame
    """

    J = utils.jacobian(q, joint)
   
    #dq_joint = dq[0, :joint]
    dq_joint = dq[:joint]
    v_and_omega = matmul(J, dq_joint)
  
    v = v_and_omega[:3]
    omega = v_and_omega[3:]
    return v, omega

