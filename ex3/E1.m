% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)
    % get total num of timesteps T.
    T = size(odo,2);
    % create state/cov arrays and set first to init.
    x_est = {}; P_est = {};
    x_est{1} = x0; P_est{1} = P0;
    % we assume nominal case where noise means are 0, since not provided.
    v_d = 0; v_th = 0; w_r = 0; w_b = 0;
    % do ekf propagation for all timesteps.
    % keep track of most recent state estimate.
    x_t = x0; P_t = P0;
    for t = 1:(T-1)
        % get landmark index.
        lm_i = zind(1,t);
        % we treat odom as command (dist, heading)^T
        u = odo(:,t); d_d = u(1); d_th = u(2);
        % compute jacobian matrices.
        F_x = [1, 0, -d_d*sin(x_t(3));
               0, 1, d_d*cos(x_t(3));
               0, 0, 1];
        F_v = [cos(x_t(3)), 0;
               sin(x_t(3)), 0;
               0, 1];
        % make prediction.
        x_predicted = [x_t(1) + (d_d + v_d)*cos(x_t(3));
                       x_t(2) + (d_d + v_d)*sin(x_t(3));
                       x_t(3) + d_th + v_th];
        P_predicted = F_x * P_t * F_x' + F_v * V * F_v';
        % check if there is a measurement on this timestep.
        if lm_i == 0
            % if no landmark meas, treat like K=0 and just use prediction.
            x_t = x_predicted;
            P_t = P_predicted;
        else
            % get true landmark position, and our rb meas of it.
            p_i = map.landmark(lm_i);
            rb = z{1,t}; %r = rb(1); b = rb(2);
            % compute remaining jacobian matrices.
            dist = sqrt((p_i(1)-x_t(1))^2 + (p_i(2)-x_t(2))^2);
            H_x = [-(p_i(1)-x_t(1))/dist, -(p_i(2)-x_t(2))/dist, 0;
                   (p_i(2)-x_t(2))/(dist^2), -(p_i(1)-x_t(1))/(dist^2), -1];
            H_w = [1, 0; 0, 1];
            % compute innovation.
            z_est = [dist; angdiff(atan2(p_i(2)-x_t(2), p_i(1)-x_t(1)) - x_t(3))];
            nu = rb - z_est - [w_r; w_b];
            % compute kalman gain.
            S = H_x * P_predicted * H_x' + H_w * W * H_w';
            K = P_predicted * H_x' * inv(S); %inv(S)
            % update step.
            x_t = x_predicted + K * nu;
            P_t = P_predicted - K * H_x * P_predicted;
        end
        % keep heading in range (-pi,pi)
        x_t(3) = angdiff(x_t(3));
        % set our estimate for time t+1.
        x_est{t+1} = x_t;
        P_est{t+1} = P_t;
    end
end
% may be helpful to use functions atan2, angdiff.
% I'm assuming from the data format that we can only observe one
  % landmark in a given timestep.
