"""
stack.py: contains all functions relevant to stacking of blocks on goal platform
"""

#Import all necessary functions
import numpy as np
from calculateIK import Main as IKMain
from dynamic_tracking import getTransformations
from time import sleep
from static_block import move_through_waypoints


def get_goal_state(lynx, color):
    """
    Function: obtain the current state of the blocks on the goal platform
    Inputs:
        - lynx: arm controller object created in final.py
        - color: string of either 'blue' or 'red'
    Outputs:
        - stack_height: height of the highest stack on goal platform  
        - goal_blocks_name: name of all blocks on goal platform
        - goal_blocks_pose: pose of all blocks on goal platform in robot frame
    """

    #Get name and pose of all blocks
    [name, pose, twist] = lynx.get_object_state()

    #Initialize lists to hold reachable static block info
    goal_blocks_name = []
    goal_blocks_pose = []
    
    #Calculate transformation matrices from world frame to robot frame
    TR_blue = np.zeros((4,4))
    TR_blue[0,0] = -1
    TR_blue[1,1] = -1
    TR_blue[2,2] =  1
    TR_blue[:,3] = np.array([200,200,0, 1])

    TR_red = np.zeros((4,4))
    TR_red[0,0] = 1
    TR_red[1,1] = 1
    TR_red[2,2] =  1
    TR_red[:,3] = np.array([200,200,0, 1])

    #Loop through all blocks
    for idx, i in enumerate(pose):
        #If block is on goal platform and we are blue
        if 450 < pose[idx][1][3] < 550 and 50 < pose[idx][0][3] < 150 and -20 < pose[idx][2][3] and color == 'blue':
            transposed_pose = np.matmul(TR_blue, pose[idx])
            goal_blocks_name.append(name[idx])
            goal_blocks_pose.append(transposed_pose)
        #If block is on goal and we are red
        if -450 > pose[idx][1][3] > -550 and -50 > pose[idx][0][3] > -150 and -20 < pose[idx][2][3] and color == 'red':
            transposed_pose = np.matmul(TR_red, pose[idx])
            goal_blocks_name.append(name[idx])
            goal_blocks_pose.append(transposed_pose)

    #Initialize highest point on goal platform
    stack_height = 0

    #Check if there are any blocks on goal
    if not goal_blocks_pose:
        #Stack height is just height of goal platform
        stack_height = 40
    else:
        #Look at z position of all blocks on goal platform and choose highest
        for pose in goal_blocks_pose:
            if stack_height == 0 or pose[2][3] > stack_height:
                stack_height = pose[2][3]
        #Assume frame of block is from its center, so add 10
        stack_height += 10

    return stack_height, goal_blocks_name, goal_blocks_pose

def stack_block(lynx, z_block, stack_height, goal_blocks_pose):
    '''
    Function: stack the block onto the highest stack on the goal platform and exit the area
    Inputs: 
        - lynx: arm controller object created in final.py
        - z_block is integer from 0-5 indicating which side the white face of the block
            is on relative to the gripper
        - stack_height: height of highest stack on goal platform
        - goal_blocks_pose: pose of all bocks on goal platform
    '''
    
    IK = IKMain()
    q = []

    #How much higher arm should be compared to goal location of block
    z_lift = 25

    #How much higher arm should be at final state (when dropping)
    z_final = 20.5

    #If no blocks on goal platform, define stack location as center of goal platform
    if not goal_blocks_pose:
        goal_platform_dir = [100, -300, 0]
    #Otherwise, define center of stack as bottom block x,y coordinates
    else:
        base_block = np.inf
        base_pose = []
        for idx, block in enumerate(goal_blocks):
            if block[2][3] < base_block:
                base_block = block[2][3]
                base_pose = block[:3, 3]
        goal_platform_dir = base_pose.copy()
        goal_platform_dir[2] = 0

    #If white is on top and the blocks aren't stacked too high
    if z_block == 0 and stack_height < 72:
        #Create target transformation matrix so end effector pointing down
        T0e_target = np.zeros((4,4))
        T0e_target[:3, 1] = goal_platform_dir/np.linalg.norm(goal_platform_dir)
        T0e_target[2,2]  = -1
        T0e_target[:3,0] = np.cross(T0e_target[:3,1],T0e_target[:3,2])
        T0e_target[3,3]  = 1
        T0e_target[:3,3] = goal_platform_dir
        T0e_target[2, 3] = stack_height + z_lift

        #Calculate all 90 degree transformations of the target
        T = getTransformations(T0e_target)

        for transform in T:
            #Figure out which is possible and save that as desired config q
            q, isPos = IK.inverse(transform)  
            if isPos:
                break
        #Set gripper to -5 to hold the block
        q[0,5] = -5

    #Otherwise stack with +y axis of gripper facing up 
    else:
        T0e_target = np.zeros((4,4))
        T0e_target[:3, 2] = goal_platform_dir/np.linalg.norm(goal_platform_dir)
        T0e_target[2,1]  = 1
        T0e_target[:3,0] = np.cross(T0e_target[:3,1],T0e_target[:3,2])
        T0e_target[3,3]  = 1
        T0e_target[:3,3] = goal_platform_dir
        T0e_target[2, 3] = stack_height + z_lift
        q, isPos = IK.inverse(T0e_target) 
        q[0,5] = -5

    #q_start is starting config before descending down to drop
    q_start = q[0]

    #Define T0e_target for final position where z position is lowered
    T0e_target_final = T0e_target
    T0e_target_final[2, 3] = stack_height + z_final
    q, isPos = IK.inverse(T0e_target_final) 
    q[0,5] = -5
    #Define final configuration to drop q
    q_final = q[0]

    #move above stack 
    move_through_waypoints(q_start, 30, lynx, True)
    #move block onto stack
    move_through_waypoints(q_final, 15, lynx,True)

    #Define configuration to open gripper
    q_open = q_final
    q_open[-1] = 30
    #opening gripper 
    move_through_waypoints(q_open, 10, lynx)
    #Define configuration to exit without knocking over stack
    q_exit = q_open
    q_exit[1] -= 0.4
    #exiting stack 
    move_through_waypoints(q_exit, 7, lynx)