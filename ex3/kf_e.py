#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv

## Global Variables ##
V = np.array([[0.01, 0],[0, 0.01]])
F=np.eye(2); G=np.eye(2); H=np.eye(2)
####

def iter_kf(x_0, P_0, u, z):
    """
    @param x_0, P_0 the prior (est. at time t)
    @param u the control input at time t
    @param z the measurement at time t+1
    @return x, P the posterior (est. at time t+1)
    """
    # prediction step.
    x_predicted = F @ x_0 + G @ u
    P_predicted = F @ P_0 @ F.T + V
    # update step.
    sig_w = (1/2)*(5-x_predicted[0,0])**2 + 0.01
    W = np.array([[sig_w, 0],[0, sig_w]])
    innovation = z - H @ x_predicted
    K_gain = P_predicted @ H.T @ inv(H @ P_predicted @ H.T + W)
    x = x_predicted + K_gain @ innovation
    P = (np.eye(2) - K_gain @ H) @ P_predicted
    return x, P

# index corresponds to timestep for u and z.
# control inputs.
u = [np.array([[1.0,-1.0]]).T,
    np.array([[2.0,-1.0]]).T,
    np.array([[-5.0,0.0]]).T]
# measurements.
z = [None,
    np.array([[3.5,-1.0]]).T,
    np.array([[5.3,-0.5]]).T,
    np.array([[-2.0,0.0]]).T]
# current state at t=0.
x = np.array([[2,2]]).T
P = np.array([[5,0],[0,5]])
# iterate to get the state at timestep 3.
for i in range(3):
    x, P = iter_kf(x, P, u[i], z[i+1])
print("x:\n", x)
print("P:\n", P)