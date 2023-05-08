# -*- coding: utf-8 -*-
"""


@author: arai
"""

import sim
import math
from scipy import spatial
import time
import random
import numpy as np
from itertools import product, combinations
from sympy.vector import CoordSys3D, gradient
from sympy import tanh, diff, lambdify, symbols
from scipy.spatial.transform import Rotation as M
from itertools import product, combinations
"uncomment as needed"
#from scipy import integrate
#from matplotlib.path import Path
#import pyny3d.geoms as pyny
#from shapely.geometry import Polygon
"import new and improved Remote API for CopelliaSim"
#from zmqRemoteApi import RemoteAPIClient
from scipy.linalg import logm, expm


import numpy as np

def rot2eul(R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

def eul2rot(theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R
    

tstep= 0.1
P=Q=R=0

"Connect to Coppeliasim"

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

errorcode, target=sim.simxGetObjectHandle(clientID,'Target',sim.simx_opmode_blocking)

errorcode, rigid=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)

theta= sim.simxGetObjectOrientation(clientID,rigid,-1,sim.simx_opmode_oneshot_wait) 
theta1= sim.simxGetObjectOrientation(clientID,target,-1,sim.simx_opmode_oneshot_wait) 

r = M.from_euler('zyx', [theta[2], theta[1], theta[0]], degrees=False)
Rr=r.as_matrix()

r = M.from_euler('zyx', [theta1[2], theta1[1], theta1[0]], degrees=False)
Rd=r.as_matrix()

Rdi= np.linalg.inv(Rd)

omega = logm(np.dot(Rdi,Rr))

errorcode = sim.simxSetObjectOrientation(clientID,rigid,-1,[-omega[2][1],-omega[0][2],-omega[1][0]],sim.simx_opmode_oneshot)