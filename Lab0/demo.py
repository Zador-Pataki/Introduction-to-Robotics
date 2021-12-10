#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
import math
from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController

q = [0,-math.pi/4,math.pi/3,-math.pi/2,0,0];
q = [0,1.4,-1.8,-1.9,-2.0,0]
q = [-math.pi/2,0,math.pi/4,0,math.pi/2,0]
q=[+0.8,1.4,-0.6,0,0,0]

q = q=[0,0,0,0,0,0]

if __name__=='__main__':

    # set up ROS interface
    con = ArmController()
    sleep(1)
    rospy.loginfo("Setup complete !")

    # send command via controller
    con.set_state(q)
    sleep(3)
    rospy.loginfo("The current state is: ")
    print(con.get_state())

    # shut down ROS interface
    con.stop()
