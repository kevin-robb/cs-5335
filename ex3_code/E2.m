% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)
    % we now assume odom readings are 100% accurate, and do pure mapping.
    indices = []; x_est = {}; P_est = {};
    x_t = []; P_t = [];
    x_v = x0; % keep track of true robot position.
    % we assume nominal case where noise means are 0, since not provided.
    w_r = 0; w_b = 0;
    T = size(odo,2); % total # of timesteps.
    for t = 1:(T-1)
        % odom gives (dist, heading)^T
        u = odo(:,t); d_d = u(1); d_th = u(2);
        % update true vehicle position.
        x_v = [x_v(1) + d_d*cos(x_v(3));
               x_v(2) + d_d*sin(x_v(3));
               x_v(3) + d_th];
        % prediction step: assume all landmarks are stationary.
        % i.e., if lm_i==0, x_t and P_t do not change this timestep.
        % => x_est(t+1) = x_est(t), P_est(t+1) = P_est(t).
        if zind(1,t) ~= 0
            % a landmark was detected this timestep.
            rb = z{1,t}; r = rb(1); beta = rb(2);
            % check if this is a new one, or one we've seen before.
            if any(indices(:) == zind(1,t))
                % this landmark is already in our state, so update it.
                i = find(zind(1,t))*2 - 1; % index of this landmark in our state.
                % get new measurement of its position.
                p_i = [x_v(1) + r*cos(x_v(3)+beta);
                     x_v(2) + r*sin(x_v(3)+beta)];
                % compute jacobian matrices.
                % interested in pseudo measurements from true vehicle
                % position x_v and the current belief (=prediction) for the
                % landmark position x_t(i:i+1).
                dist = sqrt((x_t(i)-x_v(1))^2 + (x_t(i+1)-x_v(2))^2);
                H_p = [(x_t(i)-x_v(1))/dist, (x_t(i+1)-x_v(2))/dist;
                       -(x_t(i+1)-x_v(2))/(dist^2), (x_t(i)-x_v(1))/(dist^2)];
                % pad H_x if there are multiple landmarks being tracked.
                n = size(x_t,1);
                if n < 2 % this is the only landmark.
                    H_x = H_p;
                elseif i == 1 % this is the first, not only.
                    H_x = [H_p, zeros(2,n-(i+1))]; % [H_p ...0]
                elseif i == (n-1) % this is the last, not only
                    H_x = [zeros(2,i-1), H_p]; % [0... H_p]
                else % this is somewhere in the middle. not first or last.
                    H_x = [zeros(2,i-1), H_p, zeros(2,n-(i+1))]; % [0... H_p ...0]
                end
                H_w = [1, 0; 0, 1];
                % update the state and the covariance.
                % compute innovation.
                z_est = [dist; angdiff(atan2(x_t(i)-x_v(1), x_t(i+1)-x_v(2)) - x_v(3))];
                nu = rb - z_est - [w_r; w_b];
                % compute kalman gain.
                S = H_x * P_t * H_x' + H_w * W * H_w';
                K = P_t * H_x' * inv(S);
                % update step.
                x_t = x_t + K * nu;
                P_t = P_t - K * H_x * P_t;
            else
                n = size(x_t,1);
                % this is a new landmark, so add it to our state.
                g = [x_v(1) + r*cos(x_v(3)+beta);
                     x_v(2) + r*sin(x_v(3)+beta)];
                if n < 2
                    x_t = g;
                else
                    x_t = [x_t; g];
                end
                indices = [indices; zind(1,t)]; % add landmark index.
                % compute jacobian matrices.
                G_z = [cos(x_v(3)+beta), -r*sin(x_v(3)+beta);
                       sin(x_v(3)+beta), r*cos(x_v(3)+beta)];
                % update covariance.
                if n < 2
                    P_t = G_z * W * G_z';
                else
                    Y_z = [eye(n), zeros(n,2); 
                           zeros(2,n), G_z];
                    P_t = Y_z * [P_t, zeros(n,2); zeros(2,n), W] * Y_z';
                end
            end
        end
        % set our estimate for time t+1.
        x_est{t+1} = x_t;
        P_est{t+1} = P_t;
    end
end





