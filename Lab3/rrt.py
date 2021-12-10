#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
from collections import namedtuple
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import random

def initializeTree(start, goal):
    '''
    Parameters
    ----------
    start : 1*6 numpy array
    goal : 1*6 numpy array

    Returns
    -------
    tree : List of Dictionaries(Nodes)
    '''
    
    s = {"val":start, "parents":None, "children":[]}
    tree = []
    tree.append(s)
    return tree


def inflatedObstacles(map):
    '''
    Parameters
    ----------
    filename : Path to file(.txt) that stores map

    Returns: namedTuple that contains obstacle names and inflated dimensions
    -------
    None.

    '''
    inflation = 15
    obstacles = map[0]
    boundary  = map[1]
    print("Original: ", map)
    map_min_x, map_min_y, map_min_z = boundary[:3]
    map_max_x, map_max_y, map_max_z = boundary[3:]
    inflated_obstacles=[]
    
    base = np.asarray([-40, -40, -10, 40, 40, 20])
    
    for obstacle in obstacles:
        obstacle[0] = max(map_min_x, obstacle[0]-inflation)
        obstacle[1] = max(map_min_y, obstacle[1]-inflation)
        obstacle[2] = max(map_min_z, obstacle[2]-inflation)
        obstacle[3] = min(map_max_x, obstacle[3]+inflation)
        obstacle[4] = min(map_max_y, obstacle[4]+inflation)
        obstacle[5] = min(map_max_z, obstacle[5]+inflation)
        inflated_obstacles.append(obstacle)
    

    inflated_obstacles.append(base)    
    MyStruct = namedtuple("map", "obstacles boundary")
    map = MyStruct(obstacles = inflated_obstacles, boundary = boundary)
    print("Inflated: ", map)    
    return map

def isColliding(q, map):
    '''
    Parameters
    ----------
    q : Joint Configuration 
    map : Map of Obstacles

    Returns
    -------
    Bool : Whether in Collision or not
    '''
    fk = calculateFK()
    jointPositions, T0e = fk.forward(q)
    linePt1= np.zeros((4,3))
    linePt2= np.zeros((4,3))
    for i in range(jointPositions.shape[0]-2):
        linePt1[i,:] = jointPositions[i+1,:]
        linePt2[i,:] = jointPositions[i+2,:]

    for obstacle in map[0]:
        #print(np.asarray(detectCollision(linePt1, linePt2, obstacle)))
        if(np.asarray(detectCollision(linePt1, linePt2, obstacle)).any()):
            return True
    
    return False
    
def RandomSample(map):
    '''
    Parameters
    ---------- 
    map : Map of Obstacles

    Returns
    -------
    q : Legal Random Sample in Configuration Space
    '''
    
    Found= False
    q = np.zeros(6)
    while(not Found):
        x =  np.random.uniform(low=[-1.4, -1.2, -1.8, -1.9], high=[1.4, 1.4, 1.7, 1.7], size=(4,))
        q[:4] = x
        if(not isColliding(q, map)):
            Found = True
            
    return q

def NearestNeighbour(tree, sample):
    '''
    Parameters
    ----------
    tree : RRT Tree in Configuration Space
    sample : 1*6 random sample in configuration space 

    Returns
    -------
    idx : index of nearest neighbor in tree

    '''
    idx=0
    min_dist= np.linalg.norm(tree[0]['val'] - sample)
    
    for i, node in enumerate(tree):
        dist = np.linalg.norm(node['val']-sample)
        if(dist<min_dist):
            min_dist = dist
            idx = i
    
    return idx
        

def linearinterpolation(NearestNeighbor_index, RandomSample, Tree, map):
  NearestNeighbor_branch = Tree[NearestNeighbor_index]
  NearestNeighbor_config = NearestNeighbor_branch['val']

  resolution = 50
  q = np.zeros((resolution, 6))
  q[:, 0] = np.linspace(NearestNeighbor_config[0], RandomSample[0], num = resolution)
  q[:, 1] = np.linspace(NearestNeighbor_config[1], RandomSample[1], num = resolution)
  q[:, 2] = np.linspace(NearestNeighbor_config[2], RandomSample[2], num = resolution)
  q[:, 3] = np.linspace(NearestNeighbor_config[3], RandomSample[3], num = resolution)

  path_collision = False
  for i in range(resolution):
    if isColliding(q[i, :],map):
      path_collision = True
      break
  return path_collision


def AddToTree(NearestNeighbor_index, RandomSample, Tree):
  val = RandomSample
  parents = NearestNeighbor_index
  children = []
  RandomSample_branch = {
      'val': val,
      'parents': parents,
      'children': children,
  }
  Tree.append(RandomSample_branch)
  return Tree
  
def ReachedGoal(RandomSample, Goal, Tree, map):
  tree_is_complete = False
  if not linearinterpolation(len(Tree)-1, Goal, Tree, map):
    tree_is_complete = True
    Tree = AddToTree(len(Tree)-1, Goal, Tree)
  return tree_is_complete, Tree


def RetracePath(Tree, start):
  val = Tree[-1]['val'][np.newaxis, :]
  parent = Tree[-1]['parents']
  waypoints = val
  while not parent==0:
    val = Tree[parent]['val'][np.newaxis, :]
    parent = Tree[parent]['parents']
    waypoints = np.append(waypoints, val, axis=0)

  start = np.asarray(start)[np.newaxis,:]
  waypoints = np.append(waypoints, start, axis=0)  
  waypoints = np.flip(waypoints, axis=0)
  return waypoints

def cleanup_time(tree, map):

  edit_index = len(tree)-1
  while not edit_index == 0:
    print("ei: ", edit_index)
    test_index = tree[edit_index]['parents']
    new_index = edit_index
    if(edit_index == 1):
        edit_index=0
    else:
        while not test_index == 0:
          test_index -= 1
          if not linearinterpolation(test_index, tree[edit_index]["val"],tree, map):
            new_index = test_index
          tree[edit_index]['parents'] = new_index
        edit_index = new_index
  
  return tree

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix..
    """
    #print("I Tried")
    map = inflatedObstacles(map)
    if(isColliding(start, map) or isColliding(goal, map)):
        print("Start or End not Viable")
        return []
    
    tree = initializeTree(start, goal)
    tree_is_complete = False
    while(not tree_is_complete):
        
        sample = RandomSample(map)
        NearestNeighbor_index = NearestNeighbour(tree, sample)
        if not linearinterpolation(NearestNeighbor_index, sample, tree, map):
          tree = AddToTree(NearestNeighbor_index, sample, tree)
          tree_is_complete, tree = ReachedGoal(sample, goal, tree, map)
          print("Adding more nodes")
          if tree_is_complete:
            print("Cleaning up and Retracing Path")
            tree      = cleanup_time(tree, map)
            waypoints = RetracePath(tree,start)
    
    #print(waypoints)

    resolution = 100
    linear_waypoints = np.zeros(((waypoints.shape[0]-1)*resolution,6))
    for i in range(waypoints.shape[0]-1):
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),0] = np.linspace(waypoints[i,0],waypoints[i+1,0],num=resolution)
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),1] = np.linspace(waypoints[i,1],waypoints[i+1,1],num=resolution)
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),2] = np.linspace(waypoints[i,2],waypoints[i+1,2],num=resolution)
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),3] = np.linspace(waypoints[i,3],waypoints[i+1,3],num=resolution)
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),4] = np.linspace(waypoints[i,4],waypoints[i+1,4],num=resolution)
           linear_waypoints[int(i*resolution) : int(i*resolution+resolution),5] = np.linspace(waypoints[i,5],waypoints[i+1,5],num=resolution)
    print(linear_waypoints)
    return linear_waypoints

    

def plotBox(axis, box):
    """
    :param axis: plot axis
    :param box: corners of square to be plotted
    :return: nothing
    """
    prism = Poly3DCollection([box],edgecolor='g',facecolor='g',alpha=0.5)
    axis.add_collection3d(prism)
    return    


def plot(map):
    
    fig = plt.figure()
    ax = Axes3D(fig)
        
    '''
    if detectCollision(line_pt1, line_pt2, box)[0]:
        ax.plot([line_pt1[0,0], line_pt2[0,0]], [line_pt1[0,1], line_pt2[0,1]], [line_pt1[0,2], line_pt2[0,2]], 'r')
    else:
        ax.plot([line_pt1[0,0], line_pt2[0,0]], [line_pt1[0,1], line_pt2[0,1]], [line_pt1[0,2], line_pt2[0,2]], 'b')
    '''
    for box in map[0]:
        box1 = [[box[0], box[1], box[2]],
                [box[0+3], box[1], box[2]],
                [box[0+3], box[1+3], box[2]],
                [box[0], box[1+3], box[2]]]
        box2 = [[box[0], box[1], box[2]],
                [box[0+3], box[1], box[2]],
                [box[0+3], box[1], box[2+3]],
                [box[0], box[1], box[2+3]]]
        box3 = [[box[0], box[1], box[2]],
                [box[0], box[1+3], box[2]],
                [box[0], box[1+3], box[2+3]],
                [box[0], box[1], box[2+3]]]
        box4 = [[box[0], box[1], box[2+3]],
                [box[0+3], box[1], box[2+3]],
                [box[0+3], box[1+3], box[2+3]],
                [box[0], box[1+3], box[2+3]]]
        box5 = [[box[0], box[1+3], box[2]],
                [box[0+3], box[1+3], box[2]],
                [box[0+3], box[1+3], box[2+3]],
                [box[0], box[1+3], box[2+3]]]
        box6 = [[box[0+3], box[1], box[2]],
                [box[0+3], box[1+3], box[2]],
                [box[0+3], box[1+3], box[2+3]],
                [box[0+3], box[1], box[2+3]]]
        plotBox(ax, box1)
        plotBox(ax, box2)
        plotBox(ax, box3)
        plotBox(ax, box4)
        plotBox(ax, box5)
        plotBox(ax, box6)
    plt.show()
        
#########Test Cases######
'''
start = np.array([0,  0, 0, 0, 0, 0])
goal = np.array([-1.4, -1.2, -1.8, -1.9, 0.5, 0.8])
filename = "maps/map2.txt"
map = inflatedObstacles("maps/map2.txt")
#plot(map)
print("Waypoints:", rrt(map,start,goal))
'''

