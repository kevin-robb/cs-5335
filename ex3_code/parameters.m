% Load EKF-SLAM parameters

function [V, W, x0, P0, range, fov] = parameters()
    
    % Process noise
    V = diag([0.02 0.5*pi/180].^2);
    % Sensing noise
    W = diag([0.1 1*pi/180].^2);
    % Initial mean
    x0 = [0 0 0]';
    % Initial covariance
    P0 = diag([.01 .01, 0.005].^2);
    % Maximum range of sensor
    range = 4;
    % Field of view of sensor
    fov = [-pi/2 pi/2];
    
end