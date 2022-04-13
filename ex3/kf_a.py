#!/usr/bin/env python3

# this code will perform a few iterations of a simple 1D Kalman filter using given parameters.

## Global Variables ##
sig_v = 0.04; sig_w = 0.01
f=1; g=1; h=1
####

def iter_kf(x_0, sig_0, u, z):
    """
    @param x_0, sig_0 the prior (est. at time t)
    @param u the control input at time t
    @param z the measurement at time t+1
    @return x, sig the posterior (est. at time t+1)
    """
    x_predicted = f*x_0 + g*u
    sig_predicted = f**2 * sig_0 + sig_v
    innovation = z - h * x_predicted
    k_gain = sig_predicted * h / (h**2 * sig_predicted + sig_w)
    x = x_predicted + k_gain * innovation
    sig = (1 - k_gain * h) * sig_predicted
    return x, sig

# index corresponds to timestep for u and z.
# control inputs.
u = [-0.5, 1.2, 0.3]
# measurements.
z = [None, -0.7, 0.6, 0.95]
# current state at t=0.
x = 0; sig = 1
# iterate to get the state at timestep 3.
for i in range(3):
    x, sig = iter_kf(x, sig, u[i], z[i+1])
print(x, sig)