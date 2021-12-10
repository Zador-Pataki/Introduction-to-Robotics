#!/usr/bin/python2
from copy import deepcopy
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
import numpy as np
from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController
from astar import Astar
from loadmap import loadmap
from rrt import rrt
import time

if __name__=='__main__':
    # Update map location with the location of the target map
    map_struct = loadmap("maps/map1.txt")
    
    # or run rrt code
    # start ROS
    lynx = ArmController()
    sleep(1) # wait for setup
    collision = False
    start = np.asarray(lynx.get_state()[0])
    goal = np.array([-1.4, -1.2, -1.8, -1.9,0,0])
    goal = np.array([-1.2,0,0,0,0,0])
    goal = np.array([0,1.4,0,0,0,0])
    goal = np.array([0,0,0,0,0,0])
    #goal = np.array([0.8,0.5,-0.6,0,0,0])
    #goal = np.array([0.9,1.4,-0.6,0,0,0])
    start_time = time.time()
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print("TIME to find path RRT:", time.time()-start_time)
    
    # Run Astar code
    
    start_time = time.time()
    path = Astar(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print("TIME to find path A*:", time.time()-start_time)
    
    #print(path)
    
    
    print("path: ", path)
    

    for q in path:
        print("Goal:")
        print(q)

        lynx.set_pos(q)
        reached_target = False

        # Enter user defined variables here

        while not reached_target:
            # Check if robot is collided then wait
            collision = collision or lynx.is_collided()
            #sleep(2)

            # Add Student code here to decide if controller should send next
            # target or continue to wait. Do NOT add additional sleeps to control
            # loop. You will likely want to use lynx.get_state() to decide when to
            # move to the next target.
            lynx.set_state(q)
            while np.linalg.norm(q[:5] - lynx.get_state()[0][:5])>1e-2:
                      print("Current Distance:", np.linalg.norm(q[:5] - lynx.get_state()[0][:5]))
                      print("Current State:", lynx.get_state()[0])
                      print("Setting to:", q)
                      lynx.set_state(q)
                      sleep(0.1)

            reached_target = True
            
            # End of student code

        print("Current Configuration:")
        pos, vel = lynx.get_state()
        print(pos)

    if collision:
        print("Robot collided during move")
    else:
        print("No collision detected")

    lynx.stop()
