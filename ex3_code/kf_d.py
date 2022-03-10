#!/usr/bin/env python3
import numpy as np

## Global Variables ##
sig_w = 0.01
V = np.array([[0.04, 0],[0, 0.09]])
F = np.eye(2); G = np.eye(2)
####

def iter_kf(X_0, P_0, u, z):
    """
    @param X_0, P_0 the prior (est. at time t)
    @param u the control input at time t
    @param z the measurement at time t+1
    @return X, P the posterior (est. at time t+1)
    """
    # prediction step.
    X_predicted = F @ X_0 + G @ u
    P_predicted = F @ P_0 @ F.T + V
    # compute jacobians for observation matrix. (H_w=1)
    H_x = np.array([[2*X_predicted[0,0], 2*X_predicted[1,0]]])
    # update step.
    innovation = z - (X_predicted[0,0]**2 + X_predicted[1,0]**2) # - noise?
    K_gain = P_predicted @ H_x.T / (H_x @ P_predicted @ H_x.T + sig_w).item()
    X = X_predicted + K_gain * innovation
    P = (np.eye(2) - K_gain @ H_x) @ P_predicted
    return X, P

# index corresponds to timestep for u and z.
# control inputs.
u = [np.array([[-0.5,0.3]]).T,
    np.array([[1.2,-0.6]]).T,
    np.array([[0.3,0.3]]).T]
# measurements.
z = [None, 0.6, 0.4, 1.0]
# current state at t=0.
X = np.array([[0,0]]).T
P = np.array([[1,0],[0,1]])
# iterate to get the state at timestep 3.
for i in range(3):
    # print("\nstep " + str(i) + " -> " + str(i+1))
    X, P = iter_kf(X, P, u[i], z[i+1])
print("\nPosterior Distribution for t=3:")
print("x:\n", X)
print("P:\n", P)