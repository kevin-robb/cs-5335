#!/usr/bin/env python3

# this code will perform a few iterations of a simple 2D Kalman filter using given parameters.
import numpy as np
from numpy.linalg import inv

## Global Variables ##
V = np.array([[0.04, 0],[0, 0.09]])
W = np.array([[0.01, 0],[0, 0.02]])
F=np.eye(2); G=np.eye(2); H=np.eye(2)
####

def iter_kf(x_0, P_0, u, z):
    """
    @param x_0, P_0 the prior (est. at time t)
    @param u the control input at time t
    @param z the measurement at time t+1
    @ return x, P the posterior (est. at time t+1)
    """
    x_predicted = F @ x_0 + G @ u
    P_predicted = F @ P_0 @ F.T + V
    innovation = z - H @ x_predicted
    K_gain = P_predicted @ H.T @ inv(H @ P_predicted @ H.T + W)
    x = x_predicted + K_gain @ innovation
    P = (np.eye(2) - K_gain @ H) @ P_predicted
    return x, P

# index corresponds to timestep for u and z.
# control inputs.
u = [np.array([[-0.5,0.3]]).T,
    np.array([[1.2,-0.6]]).T,
    np.array([[0.3,0.3]]).T]
# measurements.
z = [None,
    np.array([[-0.7,0.3]]).T,
    np.array([[0.6,0.0]]).T,
    np.array([[0.95,0.15]]).T]
# current state at t=0.
x = np.array([[0,0]]).T
P = np.array([[1,0],[0,1]])
# iterate to get the state at timestep 3.
for i in range(3):
    x, P = iter_kf(x, P, u[i], z[i+1])
print("x:\n", x)
print("P:\n", P)