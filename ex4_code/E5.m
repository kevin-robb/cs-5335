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
    % to create measurements for our slam node, we morph the truth with
    % noise from a known distribution, to emulate what a robot might
    % actually measure.
    zind = {}; z = {};
    % first we add noise to the odometry, regardless of sensing mode.
    n = size(odo_truth,2);
    mu_v = [0, 0]; R = chol(V);
    odo_noise = repmat(mu_v,n,1) + randn(n,2)*R;
    odo = odo_truth + odo_noise';
    % next we check which landmark(s) are visible to the robot.
    % we can do this by keeping track of the robot's current x/y/th at each
    % timestep, and computing the vector from the robot to a landmark, then
    % checking the length and angle of this vector with robot sensing
    % constraints.
    x_v = x0;
    for t = 1:n
        % propagate true veh pose forward.
        u = odo_truth(:,t); d_d = u(1); d_th = u(2);
        x_v = [x_v(1) + d_d*cos(x_v(3));
               x_v(2) + d_d*sin(x_v(3));
               x_v(3) + d_th];
        % loop through all landmarks to find which ones are visible.
        visible_landmarks = [];
        n_lm = map.nlandmarks;
        for i_lm = 1:n_lm
            % compute vector from veh position to a landmark.
            diff_vec = map.landmark(i_lm) - x_v(1:2);
            % extract range and bearing.
            r = norm(diff_vec); % range.
            b = atan2(diff_vec(2), diff_vec(1)); % global bearing.
            beta = angdiff(b - x_v(3)); % bearing relative to robot.
            % check if this is visible to robot.
            if r > range
                % out of range.
                continue
            elseif beta > fov(1) && beta < fov(2)
                % within range, and within fov.
                new_lm = [i_lm, r, beta];
                visible_landmarks = [visible_landmarks; new_lm];
            end
        end
        % set landmark measurement info for this timestep.
        if mode == 'o' % select one lm from visible at random.
            if isempty(visible_landmarks) % no landmarks are detected.
                zind{t} = 0;
                z{t} = [];
                continue
            end
            n_vis = size(visible_landmarks,1);
            i_choice = randi([1, n_vis]);
            lm_true = visible_landmarks(i_choice,:);
            % add noise to the landmark measurement.
            mu_w = [0, 0]; R = chol(W);
            lm_noise = repmat(mu_w,1,1) + randn(1,2)*R;
            lm_meas = lm_true(2:3) + lm_noise;
            % set measurements for this timestep.
            zind{t} = lm_true(1);
            z{t} = lm_meas';
        end % TODO else if other modes
    end
end