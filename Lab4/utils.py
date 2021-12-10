#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 19:54:44 2020

@author: gsuveer
"""
from calculateFK import Main
import numpy as np

fk = Main()

def velocity_jacobian(q, joint):
  T0i_list= fk.forward(q)
  
  Jv = np.zeros((3, joint+1))
  o_n = T0i_list[joint][:-1, 3]
  for i in range(joint):
    
    z_i = T0i_list[i][:-1, 2]
    o_i = T0i_list[i][:-1, 3]
    Jv[:, i] = np.cross(z_i, o_n-o_i)
    
    
    
  return Jv

def angular_velocity_jacobian(q, joint):
  T0i_list = fk.forward(q)

  Jw = np.zeros((3, joint+1))
  for i in range(joint):
    z_i = T0i_list[i][:-1, 2]
    Jw[:, i] = z_i
  return Jw

def jacobian(q, joint):
  Jv = velocity_jacobian(q, joint)
  Jw = angular_velocity_jacobian(q, joint)
  J = np.concatenate((Jv, Jw), axis=0)
  J=J[:, :-1]
  return J