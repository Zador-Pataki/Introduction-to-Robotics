"""
final.py: function to be called when starting simulation
"""

#Import all necessary functions
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from os import getcwd
sys.path.append(getcwd() + "/../Core")
from arm_controller import ArmController
from CONTROLLER import controller

if __name__=='__main__':

    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(color)
    sleep(1)

    #Wait for start gun
    lynx.wait_for_start()
    
    # Wait for setup
    qn = [0,0,0,0,0,25]
    lynx.command(qn)
    sleep(0.1) 
    
    #Call main controller function
    controller(lynx, color)
