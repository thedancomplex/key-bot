#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2015, Daniel M. Lofaro
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */


import key_bot_arm_right as kbar
import sys
import time
import math
import numpy as np
from numpy.linalg import inv
from ctypes import *
import copy


#threshold = .025
threshold = .001
alpha = .5
dT1 = .001		
dT2 = .001
dT3 = .001

#ROT TO ROT distance
lr = kbar.ROT_TO_ROT
lrx_top = kbar.TOP_ROT_TO_ROT_X
lrz_top = kbar.TOP_ROT_TO_ROT_Z
lry_bas = kbar.BAS_OFFSET_Y 
lrz_bas = kbar.BAS_OFFSET_Z
lrz_eef = kbar.EEF_OFFSET_Z
lrx_eef = kbar.EEF_OFFSET_X


# rotation matrix definition (abotu x, y, or z)
def_x = 1
def_y = 2
def_z = 3


def getRotMatrix(t, d):
    if d == def_x:
        return np.matrix([[1.0, 0.0, 0.0], [ 0.0, np.cos(t), -np.sin(t)], [ 0.0, np.sin(t), np.cos(t)]])
    elif d == def_y:
        return np.matrix([[np.cos(t), 0.0, np.sin(t)], [ 0.0, 1.0, 0.0], [ -np.sin(t), 0.0, np.cos(t)]])
    elif d == def_z:
        return np.matrix([[np.cos(t), -np.sin(t), 0.0], [ np.sin(t), np.cos(t), 0.0], [ 0.0, 0.0, 1.0]])
    else:
        return np.matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

def getTransMatrix(x,y,z):
    return np.matrix([[x], [y], [z]])
     
def getTfMatrix(R,T):    
    Ap = np.hstack([R,T])
    return np.vstack([Ap,[0.0, 0.0, 0.0, 1.0]])

def getA(a):    
#    a = [theta  , Tx  , Ty        , Tz  , def_xyz]
    R = getRotMatrix(a[0], a[4])
    T = getTransMatrix(a[1],a[2],a[3])
    A  = getTfMatrix(R,T)
    return A

def getFkArm(a, arm=None):
  m = 1.0 
  if arm == 'right':
    m = -1.0
  elif arm == 'left':
    m = 1.0
  else:
    pass

  #       theta        , Tx       , Ty          , Tz         , def_xyz
  BAS = [ a[0]         , 0.0      , lry_bas     , lrz_bas   , def_z ]
  EB0 = [ a[1]         , 0.0      , 0.0         , lr       , def_y ]
  EB1 = [ a[2]         , 0.0      , 0.0         , lr        , def_y ]
  EB2 = [ a[3]         , 0.0      , 0.0         , 0.0        , def_y ]
  EB3 = [ a[4]         , lrx_top  , 0.0         , lrz_top   , def_y ]
  EEF = [ 0.0          , lrx_eef  , 0.0         , lrz_eef   , def_x ]


  A1 = getA(BAS)
  A2 = getA(EB0)
  A3 = getA(EB1)
  A4 = getA(EB2)
  A5 = getA(EB3)
  A6 = getA(EEF)

  A = np.dot(A1,A2)
  #A = np.dot(A, A2)
  A = np.dot(A, A3)
  A = np.dot(A, A4)
  A = np.dot(A, A5)
  A = np.dot(A, A6)

#dan
#  A = np.dot(A7,A6)
#  A = np.dot(A5, A)
#  A = np.dot(A4, A)
#  A = np.dot(A3, A)
#  A = np.dot(A2, A)
#  A = np.dot(A1, A)

  return A


def getJacobian6x3(d0,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm

  # Jacobian for xyz and 6 dof
  xyz = 3 
  dof = 5
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        J[i,j] = (A1[i,3] - A0[i,3])/dt
  return J   

import multiprocessing


mp_d0 = None
mp_order = None
mp_dt = None
mp_arm = None
def getJacobianMult(d0, order, dt, arm):
  global mp_d0
  global mp_order
  global mp_dt
  global mp_arm

  mp_d0 = d0
  mp_order = order
  mp_dt = dt
  mp_arm = arm

  pool_obj = multiprocessing.Pool()
  xyz = len(order)
  J = pool_obj.map(getJacobianXYZ, range(xyz))
  return J

#def getJacobianXYZ(d0,order,dt,arm,i):
def getJacobianXYZ(i):
  global mp_d0
  global mp_order
  global mp_dt
  global mp_arm
  d0 = mp_d0
  order = mp_order
  dt = mp_dt
  arm = mp_arm
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm
  # order is the order of the desired joints [x, y, z, t_x, t_y, t_z]
  #                                          [x, y, z, t_x]
  #                                          [x, y, z, t_z]
  #                                          [x, y, z, t_y, t_z]
  #                                          Etc.

  # Jacobian or size dof x order
  xyz = len(order)
  dof = len(d0)
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

  A0 = getFkArm(d0,arm)
  #for i in range(xyz):
  for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
###        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if order[i] == 'p_x':
          J[i,j] = (A1[0,3] - A0[0,3])/dt
        elif order[i] == 'p_y':
          J[i,j] = (A1[1,3] - A0[1,3])/dt
        elif order[i] == 'p_z':
          J[i,j] = (A1[2,3] - A0[2,3])/dt
        elif order[i] == 't_x':
          a0 = getRot(A0,'x')
          a1 = getRot(A1,'x')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_y':
          a0 = getRot(A0,'y')
          a1 = getRot(A1,'y')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_z':
          a0 = getRot(A0,'z')
          a1 = getRot(A1,'z')
          J[i,j] = (a1-a0)/dt
  return J   


from parfor import parfor
def getJacobianParFor(d0,order,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm
  # order is the order of the desired joints [x, y, z, t_x, t_y, t_z]
  #                                          [x, y, z, t_x]
  #                                          [x, y, z, t_z]
  #                                          [x, y, z, t_y, t_z]
  #                                          Etc.

  # Jacobian or size dof x order
  xyz = len(order)
  dof = len(d0)
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

  A0 = getFkArm(d0,arm)
  @parfor((range(xyz)), (range(dof)))
  def fun(i, a):
####  for i in range(xyz):
###     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
###        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if order[i] == 'p_x':
          J[i,j] = (A1[0,3] - A0[0,3])/dt
        elif order[i] == 'p_y':
          J[i,j] = (A1[1,3] - A0[1,3])/dt
        elif order[i] == 'p_z':
          J[i,j] = (A1[2,3] - A0[2,3])/dt
        elif order[i] == 't_x':
          a0 = getRot(A0,'x')
          a1 = getRot(A1,'x')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_y':
          a0 = getRot(A0,'y')
          a1 = getRot(A1,'y')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_z':
          a0 = getRot(A0,'z')
          a1 = getRot(A1,'z')
          J[i,j] = (a1-a0)/dt
        return J
  return fun   

import _thread
saved_A0 = None
JJ = []
thread_num = 0
def getJacobianThread(d0,order,dt,arm):
  global saved_A0
  global JJ
  global thread_num
  thread_num = 0
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm
  # order is the order of the desired joints [x, y, z, t_x, t_y, t_z]
  #                                          [x, y, z, t_x]
  #                                          [x, y, z, t_z]
  #                                          [x, y, z, t_y, t_z]
  #                                          Etc.

  # Jacobian or size dof x order
  xyz = len(order)
  dof = len(d0)
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

  A0 = getFkArm(d0,arm)
  saved_A0 = A0
  for i in range(xyz):
    JJ.append(J)

  for i in range(xyz):
    _thread.start_new_thread(getJacobianThread, (i, J, dof,d0,dt,arm,order))
  
  while (thread_num < xyz):
    for i in range(xyz):
      J = J + JJ[i]
  return J   


def getJacobianThread(i, J, dof,d0,dt,arm,order):
     global thread_num
     global JJ
#  for i in range(xyz):
     A0 = saved_A0
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
###        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if order[i] == 'p_x':
          J[i,j] = (A1[0,3] - A0[0,3])/dt
        elif order[i] == 'p_y':
          J[i,j] = (A1[1,3] - A0[1,3])/dt
        elif order[i] == 'p_z':
          J[i,j] = (A1[2,3] - A0[2,3])/dt
        elif order[i] == 't_x':
          a0 = getRot(A0,'x')
          a1 = getRot(A1,'x')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_y':
          a0 = getRot(A0,'y')
          a1 = getRot(A1,'y')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_z':
          a0 = getRot(A0,'z')
          a1 = getRot(A1,'z')
          J[i,j] = (a1-a0)/dt
     JJ[i] = J
     thread_num = thread_num + 1

def getJacobian(d0,order,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm
  # order is the order of the desired joints [x, y, z, t_x, t_y, t_z]
  #                                          [x, y, z, t_x]
  #                                          [x, y, z, t_z]
  #                                          [x, y, z, t_y, t_z]
  #                                          Etc.

  # Jacobian or size dof x order
  xyz = len(order)
  dof = len(d0)
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

  A0 = getFkArm(d0,arm)
  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
###        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if order[i] == 'p_x':
          J[i,j] = (A1[0,3] - A0[0,3])/dt
        elif order[i] == 'p_y':
          J[i,j] = (A1[1,3] - A0[1,3])/dt
        elif order[i] == 'p_z':
          J[i,j] = (A1[2,3] - A0[2,3])/dt
        elif order[i] == 't_x':
          a0 = getRot(A0,'x')
          a1 = getRot(A1,'x')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_y':
          a0 = getRot(A0,'y')
          a1 = getRot(A1,'y')
          J[i,j] = (a1-a0)/dt
        elif order[i] == 't_z':
          a0 = getRot(A0,'z')
          a1 = getRot(A1,'z')
          J[i,j] = (a1-a0)/dt
  return J   
def getJacobian6x6(d0,dt,arm):
  # gets numerical jacobian of 'arm'
  # d0 is the current position of the arm

  # Jacobian for xyz and 6 dof
  xyz = 6
  dof = 6
  J = np.zeros((xyz,dof))
  # J[ row, col ] 
#  d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
  for i in range(xyz):
     for j in range(dof):
#        d0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        d1 = copy.deepcopy(d0)
        d1[j] = d0[j] + dt
        A0 = getFkArm(d0,arm)
        A1 = getFkArm(d1,arm)
        if i <= 2:
          J[i,j] = (A1[i,3] - A0[i,3])/dt
        else:
          if i == 3:
             a0 = getRot(A0,'x')
             a1 = getRot(A1,'x')
             J[i,j] = (a1-a0)/dt
          if i == 4:
             a0 = getRot(A0,'y')
             a1 = getRot(A1,'y')
             J[i,j] = (a1-a0)/dt
          if i == 5:
             a0 = getRot(A0,'z')
             a1 = getRot(A1,'z')
             J[i,j] = (a1-a0)/dt
  return J   


def getRot(a,ax):
  if ax == 'x':
    return np.arctan2(a[2,1],a[2,2])
  elif ax == 'y':
    return np.arctan2(-a[2,0],np.sqrt(a[2,1]*a[2,1]+a[2,2]*a[2,2]))
  elif ax == 'z':
    return np.arctan2(a[1,0], a[0,0])
  else:
    return -1  


def getDist2End(eff_current, eff_end):
  eff_vector = eff_end - eff_current # vector to end point
 
  eff_dist_to_end = np.sqrt(eff_vector[0]*eff_vector[0] + 
                            eff_vector[1]*eff_vector[1] + 
                            eff_vector[2]*eff_vector[2])
  return eff_dist_to_end

def getDist2End2(eff_current, eff_end):
  eff_vector = eff_end - eff_current # vector to end point
  
  vsum = 0.0
  for i in range(len(eff_vector)):
    vsum = eff_vector[i]*eff_vector[i] + vsum
 
  eff_dist_to_end = np.sqrt(vsum)
  return eff_dist_to_end

#print A

def getIK3dof(eff_joint_space_current, eff_end, arm=None):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z]
 # arm = 'left' or 'right' arm to solve for
 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 eff_delta_theta = 0.001 # change in goal in meters
 eff_delta_xyz = 0.001 # change in goal in meters
 eff_err_max = 0.001

 # distance to end point
 eff_dist_to_end = getDist2End(eff_current, eff_end)

 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian6x3(eff_joint_space_current, eff_delta_theta, arm)
  
  # compute inverse of jacobian
  Ji = np.linalg.pinv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff 
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta
  
  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
  eff_dist_to_end = getDist2End(eff_current, eff_end)

#  print eff_dist_to_end
 return eff_joint_space_current

def getPosCurrentFromOrder(A,order):
  J = np.zeros(len(order))
  for i in range(len(order)):
        if order[i] == 'p_x':
          J[i] = A[0,3]
        if order[i] == 'p_y':
          J[i] = A[1,3]
        if order[i] == 'p_z':
          J[i] = A[2,3]
        if order[i] == 't_x':
          a1 = getRot(A,'x')
          J[i] = a1
        if order[i] == 't_y':
          a1 = getRot(A,'y')
          J[i] = a1
        if order[i] == 't_z':
          a1 = getRot(A,'z')
          J[i] = a1
  return J   


def getIK(eff_joint_space_current, eff_end, order, arm=None, err=None, itr=None):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z,theta_x,theta_y,theta_z]
 # order is the order of the desired joints [p_x, p_y, p_z, t_x, t_y, t_z]
 #                                          [x, y, z, t_x]
 #                                          [x, y, z, t_z]
 #                                          [x, y, z, t_y, t_z]
 # arm = 'left' or 'right' arm to solve for
 # err (optional) = error for delta_theta, delta_pos (xyz) and maximum error
 # itr (optional) = number of max iterations
 if itr == None:
    itr = 100
 eff_joint_space_orig = copy.deepcopy(eff_joint_space_current)
 eff_delta_theta = 0.005 # change in goal in meters
 eff_delta_xyz = 0.005 # change in goal in meters
 eff_err_max = 0.003

 if (err is not None):
  eff_delta_theta = err[0] # change in goal in rad
  eff_delta_xyz = err[1] # change in goal in meters
  eff_err_max = err[2]  # max linear error of eef

 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = getPosCurrentFromOrder(A,order)
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 # distance to end point
 eff_dist_to_end = getDist2End2(eff_current, eff_end)
 itr_i = 0
 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian(eff_joint_space_current,order,eff_delta_theta,arm)
##  J = getJacobian6x6(eff_joint_space_current, eff_delta_theta, arm)
  
  # compute inverse of jacobian
  Ji = np.linalg.pinv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End2(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff 
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta

  # Check for over joint range
  m = 1.0
  if arm == 'left':
     m = 1.0
  elif arm == 'right':
     m = -1.0

  THE_LIM=np.pi/2.5
  print(len(eff_joint_space_current))
  if eff_joint_space_current[0] >  THE_LIM:     #xSP max
     eff_joint_space_current[0] =  THE_LIM
  if eff_joint_space_current[0] < -THE_LIM:    #xSP min
     eff_joint_space_current[0] = -THE_LIM
  if eff_joint_space_current[1] >  THE_LIM:     #xSR towards body
     eff_joint_space_current[1] =  THE_LIM      
  if eff_joint_space_current[1] < -THE_LIM:
     eff_joint_space_current[1] = -THE_LIM   
  if eff_joint_space_current[2] >  THE_LIM:     #xSY max
     eff_joint_space_current[2] =  THE_LIM
  if eff_joint_space_current[2] < -THE_LIM:    #xSY min
     eff_joint_space_current[2] = -THE_LIM
  if eff_joint_space_current[3] >  THE_LIM:     #xeP max
     eff_joint_space_current[3] =  THE_LIM
  if eff_joint_space_current[3] < -THE_LIM:    #xeP min
     eff_joint_space_current[3] = -THE_LIM
  if eff_joint_space_current[4] >  THE_LIM:     #xWY max
     eff_joint_space_current[4] =  THE_LIM
  if eff_joint_space_current[4] < -THE_LIM:    #xWY min
     eff_joint_space_current[4] = -THE_LIM
 
  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = getPosCurrentFromOrder(A,order)
  eff_dist_to_end = getDist2End2(eff_current, eff_end)
  print(itr_i, ' - ', eff_dist_to_end)
  if (itr_i >= itr):
     return (eff_joint_space_current, -1)
#     return (eff_joint_space_orig, -1)
  else:
     itr_i = itr_i+1
##  print eff_dist_to_end
 return (eff_joint_space_current, 0)


def getIK6dof(eff_joint_space_current, eff_end, arm):
 # eff_joint_space_current = [theta_SP, SR, RY, EP, WY, RR]
 # eff_end = desired end effector positon = [x,y,z,theta_x,theta_y,theta_z]
 # arm = 'left' or 'right' arm to solve for
 A = getFkArm(eff_joint_space_current,arm)
## eff_current = np.array([ A[0,3], A[1,3], A[2,3]])
 eff_current = np.array([ A[0,3], A[1,3], A[2,3], getRot(A,'x'), getRot(A,'y'), getRot(A,'z')])
# ef  = np.array([0.2, 0.2, -0.1])
## eff_end  = np.array([-0.14583 , 0.74283, 0.13834])

 eff_delta_theta = 0.001 # change in goal in meters
 eff_delta_xyz = 0.001 # change in goal in meters
 eff_err_max = 0.001

 # distance to end point
 eff_dist_to_end = getDist2End(eff_current, eff_end)

 while (eff_err_max < eff_dist_to_end):
  # jacobian of the eff_next_point
  J = getJacobian6x6(eff_joint_space_current, eff_delta_theta, arm)
  
  # compute inverse of jacobian
  Ji = np.linalg.inv(J)   # inverse

  # linear interpolation to find next point
  eff_vector = eff_end - eff_current # vector to end point
  eff_dist_to_end = getDist2End(eff_current, eff_end)
  d_eff = eff_vector/eff_dist_to_end * eff_delta_xyz
 # print d_eff 
  # compute change in DOFs d_theta = Ji * d_eff
  d_theta = np.dot(Ji,d_eff)

  # apply changes to dofs
  eff_joint_space_current = eff_joint_space_current + d_theta
  
  # distance to end point
  A = getFkArm(eff_joint_space_current,arm)
  eff_current = np.array([ A[0,3], A[1,3], A[2,3], getRot(A,'x'), getRot(A,'y'), getRot(A,'z')])
  eff_dist_to_end = getDist2End(eff_current, eff_end)

##  print eff_dist_to_end
 return eff_joint_space_current

