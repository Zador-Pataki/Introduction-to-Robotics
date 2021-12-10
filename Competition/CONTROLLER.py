"""
CONTROLLER.py: contains overarching controller functionalities,
    includes functions decision, nextBlock, and controller. 
    To be called directly from final.py. 
"""

#Import all supporting functions from other .py files 
from dynamic_tracking import dynamicAction
from stack import get_goal_state 
from static_block import split_blocks 
from static_block import which_static_block
from static_block import staticAction
from dynamic_tracking import which_dynamic_block
from stack import stack_block
from static_block import move_through_waypoints
import numpy as np

def decision(name_dynam):
    """
    Function: decide whether to pick static or dynamic block next
    Inputs:
        -name_dynam: name of dynamic cube (or empty if no dynamic cubes left)
    Outputs:
        -sDynamic: boolean, 0 if picking static block, 1 if picking dynamic block
    """
    isDynamic = False
    #If there are dynamic blocks left in pickable region, pick it up first
    if name_dynam:
        isDynamic = True
    return isDynamic

def nextBlock(lynx, color):
    """
    Function: decision function for which specific block to pick up next
    Inputs:
        - lynx: arm controller object created in final.py
        - color: string of either 'blue' or 'red'
    Outputs:
        - isDynamic: boolean, 0 if picking static block, 1 if picking dynamic
        - name (dynam or stat): name of dynamic or static cube to be picked
        - q_stat: required robot config to reach static block
        - score_stat: z-score of static cube (represents z_axis of cube)
        - pose_stat: pose of the static block
        - stack_height: height of the stack on goal platform
    """

    #Obtain names, poses and twists of all blocks on platforms separated by static vs dynamic
    static_blocks, dynamic_blocks = split_blocks(lynx, color) 

    #If there are static blocks left to be picked:
    if len(static_blocks):
        #Calculate current stack height on goal platform
        stack_height, _ , _= get_goal_state(lynx, color)
        #Calculate the best static block to pick next
        q_stat, score_stat, name_stat, pose_stat = which_static_block(static_blocks, stack_height)

    #If there are dynamic blocks left to be picked:
    if len(dynamic_blocks[0]):
        #Define upper and lower bounds for where on dynamic platform is reachable
        theta_lb_p = -np.pi/8
        theta_ub_p = 3*np.pi/8
        radius = 5
        theta_lb_o = -5*np.pi/8
        theta_ub_o = theta_ub_p
        
        #Get the name of the best dynamic block to pick next
        name_dynam = which_dynamic_block(dynamic_blocks, theta_lb_p, theta_ub_p, theta_lb_o, theta_ub_o, radius, color)
    
    #If there are both dynamic and static blocks, choose the dynamic one
    if len(dynamic_blocks[0]) and len(static_blocks):
        isDynamic = decision(name_dynam)
    #Otherwise pick whichever type of block is still pickable
    elif len(dynamic_blocks[0]):
        isDynamic = True
    elif len(static_blocks):
        isDynamic = False
    
    #If we are picking dynamic block next, return dynamic block info
    if isDynamic:
        return isDynamic, name_dynam, None, None, None, stack_height
    #Else return relevant static block info
    else:
        return isDynamic, name_stat, q_stat, score_stat, pose_stat, stack_height


def controller(lynx, color):
    """
    Function: main controller function for choosing blocks, picking, and stacking
    Inputs:
        - lynx: arm controller object created in final.py
        - color: string of either 'blue' or 'red'
    """

    #Run continuously 
    while True:
        #Choose the next block to be picked
        isDynamic, name, q, score, pose, stack_height = nextBlock(lynx, color)
        print("Picking Block: ", name, " ", isDynamic)

        #If no more blocks to be picked next, continue
        if len(name) == 0:
            continue
        
        #If picking up dynamic block
        if isDynamic:
            success = dynamicAction(lynx, color, name)
        #If picking up static block
        else:
            success = staticAction(lynx, q, name, pose, color)
        #If picking up the cube was successful
        if success:
            #Define number of waypoints based on what type of block was picked
            if isDynamic:
                linspace = 25
            else:
                linspace = 10
            #Move to the goal platform and stack
            move_through_waypoints([-1,0,0,0,0,-5], linspace, lynx, True)
            stack_block(lynx, score, stack_height, None)     
