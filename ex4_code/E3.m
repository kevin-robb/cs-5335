% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
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

function [x_est, P_est, indices] = E3(odo, zind, z, V, W, x0, P0)
% This is the same file I submitted for E0 for ex3.
    % we now don't know the true veh pose or the map. SLAM time.
    indices = []; x_est = {}; P_est = {};
    x_est{1} = x0; P_est{1} = P0;
    x_t = x0; P_t = P0;
    % we assume nominal case where noise means are 0.
    v_d = 0; v_th = 0; w_r = 0; w_b = 0;
    T = size(odo,2); % total # of timesteps.
    for t = 1:(T-1)
        % odom gives (dist, heading)^T "command"
        u = odo(:,t); d_d = u(1); d_th = u(2);
        % compute jacobian matrices.
        F_xv = [1, 0, -d_d*sin(x_t(3));
                0, 1, d_d*cos(x_t(3));
                0, 0, 1];
        F_x = eye(size(x_t,1));
        F_x(1:3,1:3) = F_xv;
        F_vv = [cos(x_t(3)), 0;
               sin(x_t(3)), 0;
               0, 1];
        F_v = zeros(size(x_t,1),2);
        F_v(1:3,1:2) = F_vv;
        % make prediction. (only the veh position is expected to change)
        x_predicted = x_t;
        x_predicted(1) = x_t(1) + (d_d + v_d)*cos(x_t(3));
        x_predicted(2) = x_t(2) + (d_d + v_d)*sin(x_t(3));
        x_predicted(3) = x_t(3) + d_th + v_th;
        % predict covariance.
        P_predicted = F_x * P_t * F_x' + F_v * V * F_v';
        % update step, using landmark measurements.
        if zind(1,t) ~= 0
            % a landmark was detected this timestep.
            rb = z{1,t}; r = rb(1); beta = rb(2);
            % check if this is a new one, or one we've seen before.
            if any(indices(:) == zind(1,t))
                % this landmark is already in our state, so update it.
                i = find(indices == zind(1,t))*2 - 1 + 3; % index of this landmark in our state.
                % get new measurement of its position.
%                 p_i = [x_predicted(1) + r*cos(x_predicted(3)+beta);
%                        x_predicted(2) + r*sin(x_predicted(3)+beta)];
                % compute jacobian matrices.
                % interested in pseudo measurements from predicted vehicle
                % position x_predicted and the current belief (=prediction) for the
                % landmark position x_t(i:i+1).
                dist = sqrt((x_t(i)-x_predicted(1))^2 + (x_t(i+1)-x_predicted(2))^2);
                H_xv = [-(x_t(i)-x_predicted(1))/dist, -(x_t(i+1)-x_predicted(2))/dist, 0;
                        (x_t(i+1)-x_predicted(2))/(dist^2), -(x_t(i)-x_predicted(1))/(dist^2), -1];
                H_xp = [(x_t(i)-x_predicted(1))/dist, (x_t(i+1)-x_predicted(2))/dist;
                       -(x_t(i+1)-x_predicted(2))/(dist^2), (x_t(i)-x_predicted(1))/(dist^2)];
                H_x = zeros(2,size(x_t,1));
                H_x(1:2,1:3) = H_xv;
                H_x(1:2,i:i+1) = H_xp;
                H_w = [1, 0; 0, 1];
                % update the state and the covariance.
                % compute innovation.
                z_est = [dist; angdiff(atan2(x_t(i+1)-x_predicted(2), x_t(i)-x_predicted(1)) - x_predicted(3))];
                nu = rb - z_est - [w_r; w_b];
                % compute kalman gain.
                S = H_x * P_predicted * H_x' + H_w * W * H_w';
                K = P_predicted * H_x' * inv(S);
                % update step.
                x_t = x_predicted + K * nu;
                P_t = P_predicted - K * H_x * P_predicted;
            else
                n = size(x_t,1);
                % this is a new landmark, so add it to our state.
                g = [x_predicted(1) + r*cos(x_predicted(3)+beta);
                     x_predicted(2) + r*sin(x_predicted(3)+beta)];
                x_t = [x_predicted; g]; % add to state.
                indices = [indices; zind(1,t)]; % add landmark index.
                % compute jacobian matrices.
                G_z = [cos(x_predicted(3)+beta), -r*sin(x_predicted(3)+beta);
                       sin(x_predicted(3)+beta), r*cos(x_predicted(3)+beta)];
                Y_z = eye(n+2);
                Y_z(n+1:n+2,n+1:n+2) = G_z;
                % update covariance.
                P_t = Y_z * [P_predicted, zeros(n,2); zeros(2,n), W] * Y_z';
            end
        else
            % no landmark was detected, so we have no measurement to use.
            x_t = x_predicted;
            P_t = P_predicted;
        end
        % set our estimate for time t+1.
        x_est{t+1} = x_t;
        P_est{t+1} = P_t;
    end
end