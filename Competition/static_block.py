"""
static_block: contains all functions relevant to picking and choosing 
    static blocks from the static platforms
"""

#Import all necessary functions
import numpy as np
from calculateIK import Main as IKMain
from calculateFK import Main as FKMain
from dynamic_tracking import getTransformations
from time import sleep

def split_blocks(lynx, color):
    '''
    Function: Split all non-goal blocks into dynamic vs. static
    Inputs:
        - lynx: arm controller object created in final.py
        - color: string of either 'blue' or 'red'
    Outputs: 
        - static_blocks: list of 2 lists: static names and static poses
        - dynamic_blocks: list of 3 lists: dynamic names, dynamic poses, dynamic twists
    '''

    [name, pose, twist] = lynx.get_object_state()

    #Initialize lists to hold static and dynamic block info
    static_blocks = []
    dynamic_blocks = []

    #Initialize lists to hold name, pose, twist info
    static_name = []
    static_pose = []
    dynamic_name = []
    dynamic_pose = []
    dynamic_twist = []

    #Define transformation matrices from world frame to robot frame
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

    #Loop through all twist
    for idx, i in enumerate(twist):
        #Check if static block
        if not name[idx].find("static")==-1:
            #If y coordinate of block is positive and we are blue and not on goal or on floor
            if 450 > pose[idx][1][3] > 0 and -20 < pose[idx][2][3] and color == 'blue':
                transposed_pose = np.matmul(TR_blue, pose[idx])
                static_name.append(name[idx])
                static_pose.append(transposed_pose)
            #If y coordinate of block is negative and we are red and not on goal or on floor
            if -450 < pose[idx][1][3] < 0 and -20 < pose[idx][2][3] and color == 'red':
                transposed_pose = np.matmul(TR_red, pose[idx])
                static_name.append(name[idx])
                static_pose.append(transposed_pose)
        #Otherwise it's dynamic
        else:
            #Check if still on the rotating table
            if -20 < pose[idx][2][3] < 60.5 and abs(pose[idx][1][3])<400:
                dynamic_name.append(name[idx])
                dynamic_pose.append(pose[idx])
                dynamic_twist.append(twist[idx])

    static_blocks.append(static_name)
    static_blocks.append(static_pose)
    dynamic_blocks.append(dynamic_name)
    dynamic_blocks.append(dynamic_pose)
    dynamic_blocks.append(dynamic_twist)

    return static_blocks, dynamic_blocks

def which_static_block(static_blocks, stack_height):
    '''
    Function: Analyze all static blocks and figure out which one is the best to pick up next
    Inputs:
        - static_blocks: list of 2 lists: static names and static poses
        - stack_height: height of tallest stack on goal platform
    Outputs:
        - q_stat: 1x6 list of joint angles of robot needed to move to ideal static block
        - score_stat: integer 0-5 for which side the white side of cube is facing (output from white_side)
        - name_stat: name of static cube to be picked
        - pose_stat: pose of cube to be picked
    '''
    static_names = static_blocks[0]
    static_poses = static_blocks[1]

    #Get configs of cubes that can be picked from top and those that cannot
    q_list, z_list, can_pick_name, can_pick_pose, cannot_pick_name, cannot_pick_pose = can_pick_static_top(static_names, static_poses)

    #z_lift (additional height before stack) + stack_height < 98
    #We define z_lift as 25 so stack height must be less than 72
    if stack_height < 72:
        try:
            #z_list = 0 is where white side is facing up
            idx = np.argmin(z_list)
            q_stat = q_list[idx][0]
            score_stat = z_list[idx]
            name_stat = can_pick_name[idx]
            pose_stat = can_pick_pose[idx]
        except:
            #Otherwise try again to see if we get more static cubes 
            q_list, z_list, can_pick_name, can_pick_pose, cannot_pick_name, cannot_pick_pose = can_pick_static_top(static_names, static_poses)
    #If stack height > 72, ideal to pick a block where white is not on top
    else:
        #Find minimum value that is not 0
        if z_list and not all(elem == 0 for elem in z_list):
            m = min(i for i in z_list if i > 0)
            idx = z_list.index(m)
            q_stat = q_list[idx][0]
            score_stat = z_list[idx]
            name_stat = can_pick_name[idx]
            pose_stat = can_pick_pose[idx]
        #If only white side facing up is left, then pick it up
        elif z_list and all(elem == 0 for elem in z_list):
            idx = np.argmin(z_list)
            q_stat = q_list[idx][0]
            score_stat = z_list[idx]
            name_stat = can_pick_name[idx]
            pose_stat = can_pick_pose[idx]
        #If there are no blocks to be picked from top, calcualte pose for picking up from side
        else:
            q, new_name, new_pose = cannot_pick_static_top(cannot_pick_name, cannot_pick_pose)
            q_stat = q
            score_stat = [] 
            name_stat  = new_name
            pose_stat  = new_pose

    return q_stat, score_stat, name_stat, pose_stat

def cannot_pick_static_top(blocks_name, blocks_pose):
    """
    Function:
    Inputs:
        - blocks_name: names of all static blocks that can't be picked up from the top
        - blocks_pose: poses of all static blocks that can't be picked up from the top
    Outputs:
        - q: configuration required to pick cube up from the side
        - new_name: name of cube to be picked up
        - new_pose: pose of cube to be picked up
    """
    #Figure out which block to pick up first
    pose_dist = np.inf
    new_pose = []
    new_name = []
    IK = IKMain()
    #Loop through all cannot pick poses and pick closest one in distance
    for idx, imp_pose in enumerate(blocks_pose):
        #If white side facing up, pick that up first
        if imp_pose[2][2] == 1:
            new_name = blocks_name[idx]
            new_pose = imp_pose
            break
        #Otherwise choose the cube that is closest
        dist = np.linalg.norm(imp_pose[:3, 3])
        if dist < pose_dist:
            pose_dist = dist
            new_name = blocks_name[idx]
            new_pose = blocks_pose[idx]
    #Set goal direction as the x, y position of the cube
    goal_dir = new_pose[:3, 3]
    goal_dir[2] = 0

    #Get transformation matrix of end effector to get to goal direction
    T0e_target = np.zeros((4,4))
    T0e_target[:3, 2] = goal_dir/np.linalg.norm(goal_dir)
    T0e_target[2,1]  = 1
    T0e_target[:3,0] = np.cross(T0e_target[:3,1],T0e_target[:3,2])
    T0e_target[3,3]  = 1
    T0e_target[:3,3] = goal_dir
    T0e_target[2, 3] = new_pose[2, 3]
    q, isPos = IK.inverse(T0e_target) 
    #If configuration is possible, open gripper and save configuration q
    if isPos:
        q[0,5] = 25
        q = q[0].copy()
    else:
        q = []
        new_name = []
        new_pose = []

    return q, new_name, new_pose

def can_pick_static_top(our_blocks_name, our_blocks_pose):
    '''
    Function: Determine which static blocks we can feasibly pick up from the top 
    Inputs:
        - our_blocks_name: names of all static blocks on static platforms
        - our_blocks_pose: poses of all static blocks on static platforms
    Outputs:
        - q_list: list of all configurations for robot corresponding to each block
        - z_list: integer 0-5 for which side the white side of cube is facing 
        - can_pick_name: list of all names of cubes that can be picked from top
        - can_pick_pose: list of poses of all cubes that can be picked from top
        - cannot_pick_name: list of all names of cubes that cannot be picked from top
        - cannot_pick_pose: list of all poses of cubes that cannot be picked from top
    '''
    #How much higher arm should be compared to block
    z_lift = 40

    IK = IKMain()

    q_list = []
    z_list = []
    can_pick_name = []
    can_pick_pose = []
    cannot_pick_name = []
    cannot_pick_pose = []

    #Loop through all blocks
    for idx, curr_pose in enumerate(our_blocks_pose):

        #Figure out which axis of block is facing up
        z_axis = np.matmul(curr_pose[:3,:3].T, np.array([0,0,-1]).reshape(-1,1))
        top_coord = np.argmax(np.abs(z_axis))

        #Indices of all non-z-aligned column
        indices = np.ones(4, dtype = np.int)
        indices[top_coord] = 0

        #Create T0e_target for robot to match block
        block_xy = curr_pose[:3,indices]
        T0e_target = np.zeros((4,4))
        T0e_target[2,2]  = -1
        T0e_target[:3,0] = np.cross(block_xy[:,1],T0e_target[:3,2])
        T0e_target[:3,1] = block_xy[:,1]
        T0e_target[3,3]  = 1
        T0e_target[:3,3] = curr_pose[:3,3]
        T0e_target[2,3] += z_lift

        #Calculate all 90 degree transformations of the target
        T = getTransformations(T0e_target)

        #Variable to show whether we gripped not on the white side
        success_grip = 0
        success_transform = 0
        #Initialize variables to hold potentially successful configs
        hold_q = []
        hold_transform = []

        #Block's z axis
        block_z = curr_pose[:3, 2]

        isPoscount = 0

        #Loop through all 90 degree transformation matrices
        for transform in T:
            #Check for whether transformation matrix is achievable
            q, isPos = IK.inverse(transform)
            if isPos:
                isPoscount += 1
                #Get the y and x axes of the end effector
                x_axis = transform[:3, 0]
                y_axis = transform[:3, 1]
                #If gripper is not on the x axis and white side aligns with +y axis of end effector (desirable for stacking)
                if np.around(np.dot(x_axis, block_z), 8) == 0 and np.dot(y_axis, block_z) > 0:
                    success_grip = 1
                    success_transform = transform
                    break
                #If gripper is not on x axis but white side aligns with -y axis of gripper
                elif np.around(np.dot(x_axis, block_z), 8) == 0 and np.dot(y_axis, block_z) < 0:
                    hold_q = q
                    hold_transform = transform
                #If gripper is on the x axis and hold_q is empty 
                elif np.around(np.dot(x_axis, block_z), 8) != 0 and len(hold_q) == 0:
                    hold_q = q
                    hold_transform = transform

        #If no successful grip but there is a isPos = 1 grip
        if success_grip == 0 and len(hold_q) != 0:
            q = hold_q
            success_transform = hold_transform

        q[0,5] = 23

        #If at least one of the transformations was possible
        if isPoscount != 0:
            q_list.append(q)
            can_pick_name.append(our_blocks_name[idx])
            can_pick_pose.append(our_blocks_pose[idx])
            z = white_side(curr_pose, success_transform)
            z_list.append(z)

        #If none of the transformations were possible
        if isPoscount == 0:
            cannot_pick_name.append(our_blocks_name[idx])
            cannot_pick_pose.append(our_blocks_pose[idx])

    return q_list, z_list, can_pick_name, can_pick_pose, cannot_pick_name, cannot_pick_pose


def white_side(block_pose, success_transform):
    '''
    Function: output a numerical value which shows how the +z axis of the block (white side) aligns with gripper
    Inputs:
        - block_pose: pose of the block in consideration
        - success_transform: transformation matrix of end effector to get to block
    Outputs:
        - Z = 0: z axis of block aligns with -z axis of end effector
        - Z = 1: z axis of block aligns with +y axis of end effector
        - Z = 2: z axis of block aligns with -y axis of end effector
        - Z = 3: z axis of block aligns with +x axis of end effector
        - Z = 4: z axis of block aligns with -x axis of end effector
        - Z = 5: z axis of block aligns with +z axis of end effector
    '''
    #Define which axis of block faces +z direction
    z_axis = np.matmul(block_pose[:3,:3].T, np.array([0,0,1]).reshape(-1,1))
    top_coord = np.argmax(np.abs(z_axis))
    block_z = block_pose[:3, 2] #Z axis of block
    x_axis = success_transform[:3, 0] #X axis of gripper
    y_axis = success_transform[:3, 1] #Y axis of gripper

    #If Z is facing white side up (negative of end effector's z)
    if top_coord == 2 and z_axis[top_coord] > 0:
        z = 0
    #If Z is facing white side down (same as end effector z)
    elif top_coord == 2 and z_axis[top_coord] < 0:
        z = 5
    #If gripper is not on the x axis and white side aligns with +y axis of end effector
    elif np.around(np.dot(x_axis, block_z), 8) == 0 and np.dot(y_axis, block_z) > 0:
        z = 1
    #If gripper is not on x axis but white side aligns with -y axis of gripper
    elif np.around(np.dot(x_axis, block_z), 8) == 0 and np.dot(y_axis, block_z) < 0:
        z = 2
    #If gripper is aligned with +x axis of gripper
    elif np.around(np.dot(y_axis, block_z), 8) == 0 and np.dot(x_axis, block_z) > 0:
        z = 3
    #If gripper is aligned with -x axis of gripper
    elif np.around(np.dot(y_axis, block_z), 8) == 0 and np.dot(x_axis, block_z) < 0:
        z = 4

    return z

def staticAction(lynx, q, cube_name, cube_pose, color):
    '''
    Function: Pick up a static cube
    Inputs:
        - lynx: arm controller object created in final.py
        - q: configuration of robot to go right above the cube
        - cube_name: name of cube to be picked up
        - cube_pose: pose of cube to be picked up
        - color: string of either 'blue' or 'red'
    Outputs: 
        - success: 0 if unsuccessful pick up, 1 if successful pick up
    '''
    #Moves robot right above block
    move_through_waypoints(q, 15, lynx, flag=False, sleep_timer=0.1)

    #Define transformation matrices from world frame to robot frame
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
    
    if color == "blue":
        robot_frame = TR_blue
    elif color == "red":
        robot_frame = TR_red
    
    success = 0
    FK = FKMain()
    IK = IKMain()
    T0i, jointPositions = FK.forward(q)
    #Isolate T0e
    T0e_target = T0i[5]
    #Drop the arm 10 more mm (to the block)
    T0e_target[2, 3] -= 30
    q_new, isPos = IK.inverse(T0e_target) 
    q_new[0,5] = 30
    if isPos:
        #move down on the cube
        move_through_waypoints(q_new[0], 10, lynx) 

        #Define new configuration to close gripper
        q_new[0,-1] = -5

        #close the gripper
        move_through_waypoints(q_new[0], 1, lynx, True)
        
        #Define new configuration where joint 1 does not change but everything else returns to 0        
        q_new = [q_new[0,0],0,0,0,0,-5]
        move_through_waypoints(q_new, 5, lynx, True)

    #Get all object states again
    [name, pose, twist] = lynx.get_object_state()
    idx = name.index(cube_name)
    #Check that z position of cube is now above the static platform
    if pose[idx][2][3] > 75:
        success = 1
    return success

def move_through_waypoints(q_end, lin_space_num, lynx, flag=False, sleep_timer=0.1):
    """
    Function: Calculate waypoints for the robot to move between 2 configurations
    Inputs:
        - q_end: desired ending configuration for robot
        - lin_space_num: number of waypoints between current and goal configuration
        - lynx: arm controller object created in final.py
        - flag: boolean, 1 if holding a block currently, 0 if not holding block
        - sleep_timer: amount of time in seconds to call sleep for between commanding configurations
    """
    #Calculate robot's current configuration as starting config
    [q_start, _] = lynx.get_state()
    
    #If holding a block, put gripper to -5 to maintain contact
    if flag:
        q_start[-1] = -5
    
    q_path = np.zeros((lin_space_num, 6))

    #Create linearly spaced waypoints for each joint
    for i in range(len(q_start)):
        q_path[:, i] = np.linspace(q_start[i], q_end[i], lin_space_num).T
    
    #Command the robot to go to each of the waypoints
    for i in range(q_path.shape[0]):
        lynx.command(q_path[i, :])
        sleep(sleep_timer)
