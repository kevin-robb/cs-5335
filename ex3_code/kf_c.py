#!/usr/bin/env python3

# this code will perform a few iterations of a simple 2D Kalman filter using given parameters and velocity motion commands.
import numpy as np

## Global Variables ##
dt = 1
vel_noise_mu = 0; vel_noise_sig = 0.03
pos_noise_mu = 0; pos_noise_sig = 0.02
sig_w = 0.01
noise_mu = np.array([[pos_noise_mu,vel_noise_mu]]).T
V = np.array([[pos_noise_sig, 0],[0, vel_noise_sig]])
# W = np.array([[sig_w, 0],[0, sig_w]])
F = np.array([[1, dt], [0, 1]])
G = np.array([[0, 1]]).T
H = np.array([[1, 0]])
####

def iter_kf(X_0, P_0, u, z):
    """
    @param X_0, P_0 the prior (est. at time t)
    @param u the control input at time t
    @param z the measurement at time t+1
    @return X, P the posterior (est. at time t+1)
    """
    X_predicted = F @ X_0 + G*u + noise_mu
    P_predicted = F @ P_0 @ F.T + V
    innovation = z - H @ X_predicted
    K_gain = P_predicted @ H.T / (H@P_predicted@H.T + sig_w).item()
    X = X_predicted + K_gain * innovation
    P = (np.eye(2) - K_gain @ H) @ P_predicted
    return X, P

# index corresponds to timestep for u and z.
# control inputs.
u = [-0.2, 0.0, 0.1]
# measurements.
z = [None, 0.4, 0.9, 0.8]
# current state at t=0.
X = np.array([[0,0]]).T
P = np.array([[1,0],[0,1]])
# iterate to get the state at timestep 3.
for i in range(3):
    X, P = iter_kf(X, P, u[i], z[i+1])
print("x:\n", X)
print("P:\n", P)