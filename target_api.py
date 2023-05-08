# -*- coding: utf-8 -*-
"""

@author: arai
"""



import math
import sim
import scipy
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


"initialize 100 random rotation matrices and find the karcher mean by averaging on the manifold"

e=10**(-44)
l=0
r=0
k=100

R = np.zeros((k, 3, 3))


for j in range(k):
    r1 = M.from_euler('zyx', [random.uniform(0,1.4), random.uniform(0,1.4), random.uniform(0,1.4)], degrees=False)
    R[j,:,:]= r1.as_matrix()

r1 = M.from_euler('zyx', [0.1, 0.1, 0.1], degrees=False)
S=r1.as_matrix()

while(l<1):
    i=0
    r=0
    for i in range(k):
        r=r+ logm(np.dot(np.linalg.inv(S),R[j,:,:]))
    
    r=r/k
    print(np.linalg.det(r))
    if abs(np.linalg.det(r))<e:
        l=1
    else:
        S=np.dot(S,expm(r))

tstep= 0.1
P=Q=R=0

"Connect to Coppeliasim"

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
t=3


"getting all the handles on the objects in the scene"

errorCode, vision=sim.simxGetObjectHandle(clientID,'vision',sim.simx_opmode_blocking)
errorCode, camera=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
errorCode, left_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
errorCode, right_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
errorCode, p3dx=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
errorCode, UGV=sim.simxGetObjectHandle(clientID,'Dummy0',sim.simx_opmode_blocking)
errorCode, origin=sim.simxGetObjectHandle(clientID,'Dummy1',sim.simx_opmode_blocking)
errorCode, perturbation=sim.simxGetObjectHandle(clientID,'Dummy2',sim.simx_opmode_blocking)

errorCode=sim.simxSetJointTargetVelocity(clientID, left_motor,0,sim.simx_opmode_streaming)
errorCode=sim.simxSetJointTargetVelocity(clientID, right_motor,0,sim.simx_opmode_streaming)

while (t) < 10000000000:
    
    errorcode,  theta= sim.simxGetObjectOrientation(clientID,camera,p3dx,sim.simx_opmode_oneshot_wait) 
    errorcode,  theta1= sim.simxGetObjectOrientation(clientID,UGV,-1,sim.simx_opmode_oneshot_wait) 
    
    r1 = M.from_euler('zyx', [theta1[2], 0, 0], degrees=False)
    Rgi=r1.as_matrix()
    Rig = np.linalg.inv(Rgi)
    
    r = M.from_euler('zyx', [theta[2], theta[1], theta[0]], degrees=False)
    Rcg=r.as_matrix()
    Rgc = np.linalg.inv(Rcg)
    
    Ric=Rig*Rgc
    Rci = np.linalg.inv(Ric)
    
    c11=Rcg[0][0]
    c12=Rcg[0][1]
    c13=Rcg[0][2]
    c21=Rcg[1][0]
    c22=Rcg[1][1]
    c23=Rcg[1][2]
    c31=Rcg[2][0]
    c32=Rcg[2][1]
    c33=Rcg[2][2]
    
    St= np.dot(Rig,S)
    St= np.linalg.inv(St)
    
    d11=St[0][0]
    d12=St[0][1]
    d13=St[0][2]
    d21=St[1][0]
    d22=St[1][1]
    d23=St[1][2]
    d31=St[2][0]
    d32=St[2][1]
    d33=St[2][2]
    
    p= -1/2*(c12*d31 + c22*d32 + c32*d33-(c13*d21 + c23*d22 + c33*d23))
    q= -1/2*(c13*d11 + c23*d12 + c33*d13-(c11*d31 + c21*d32 + c31*d33))
    r= -1/2*(c11*d21 + c21*d22 + c31*d23-(c12*d11 + c22*d12 + c32*d13))
    
    P=P+tstep*p
    Q=Q+tstep*q
    R=R+tstep*r
    
    errorCode= sim.simxSetObjectOrientation(clientID,camera,UGV,[float(P),float(Q),float(R)],sim.simx_opmode_oneshot)
    print(S)
    time.sleep(tstep)
    
    
    
    
    