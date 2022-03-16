% Input: odo_truth -> 2xT matrix containing true (non-noisy)
%                     odometry readings for T time steps
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks)
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
%        mode -> Character denoting sensing mode
%                'o': One visible landmark (if any) detected per time step
%                'a': All visible landmarks (if any) detected per time step
%                'f': False positives detections are added to observations
% Output: odo -> 2xT matrix containing noisy odometry readings for T time steps
%         zind -> 1xT cell array containing the observed landmark indices
%                 for T time steps; zind{t} is empty if no landmarks observed
%         z -> 1xT cell array containing the (range, bearing) observation
%              for T time steps; z{t} is empty if no observation at time t

function [odo, zind, z] = E5(odo_truth, map, V, W, x0, range, fov, mode)

end