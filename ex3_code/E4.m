% Input: V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
% Output: ekf_s -> Robotics toolbox EKF object after performing SLAM
%                  (with double maximum sensing range R = 8)

function ekf_s = E4(V, W, x0, P0, range, fov)
    % SLAM code directly copied from E0.
    rng(0);
    map = LandmarkMap(20);
    veh = Bicycle('covar', V, 'x0', x0);
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', fov, 'range', range, 'animate');
    ekf_s = EKF(veh, V, P0, sensor, W, []);
    ekf_s.run(1000);
    % plot stuff.
%     map.plot();
%     ekf_s.plot_map('g');
%     ekf_s.plot_xy('r');
%     veh.plot_xy('b');
end