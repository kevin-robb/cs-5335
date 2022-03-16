% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT cell array containing the observed landmark index for
%                T time steps; zind{t} is empty if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 1x1 matrix denoting the sensing noise in (range)
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first
%                    three rows of x_est and P_est correspond to the
%                    vehicle state, the next two correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E9(odo, zind, z, V, W, range, fov, x0, P0)

end