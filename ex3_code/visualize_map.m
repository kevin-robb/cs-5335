% Visualizes the estimated landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Returns error statistics between each pair of comparisons.

% Input: x_est -> 1xT cell array containing the map state mean
%                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                 where M is the number of landmarks observed by time t)
%        P_est -> 1xT cell array containing the vehicle state covariance
%                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                 where M is the number of landmarks observed by time t)
%        indices -> Mx1 vector containing the landmark index corresponding
%                   to the entries in the state vector, where M is the
%                   number of landmarks observed by the final time step T)
%                   For example, if indices is [15; 4], then the first
%                   three rows of x_est and P_est correspond to the
%                   vehicle state, the next two correspond to landmark 15,
%                   and the next two rows correspond to landmark 4, etc.
%        map -> Robotics toolbox Map object containing the
%               known map (known landmarks)
%        is_slam -> Boolean value, true if in SLAM mode,
%                   false if in mapping mode with known vehicle poses
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        is_plot -> Boolean value; map is visualized if and only if true
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
% Output: [err_xt, err_et, err_xe, num_xt, num_et, num_xe]
%         There are 2 triples of errors, for landmark position error
%         and number of landmarks detected respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function [err_xt, err_et, err_xe, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, is_slam, sol, is_plot, compute_errors)
    % If doing SLAM, need to offset indices by 3
    % (first 3 elements of state vector are for vehicle position estimate)
    if is_slam
        offset = 3;
    else
        offset = 0;
    end

    % Collect the estimated landmark positions
    T = length(x_est);
    M = (length(x_est{T}) - offset) / 2;
    m_est = reshape(x_est{T}((1+offset):end), [2 M]);
    if is_plot
        % Plot the estimated landmarks as red dots
        scatter(m_est(1,:), m_est(2,:), 'r.');
        % Plot 95%-confidence ellipses around landmarks
        for i = 1:M
            plot_ellipse(P_est{T}((2*i-1+offset):(2*i+offset),(2*i-1+offset):(2*i+offset)) * 5.991, m_est(:,i), 'r');
        end
    end
    
    % If toolbox EKF object is passed in, visualize the estimate too
    ekf_exists = ~isempty(sol);
    if ekf_exists
        % Collect and plot the estimated landmarks in green
        M_est = (length(sol.history(T).x_est) - offset) / 2;
        e_est = reshape(sol.history(T).x_est((1+offset):end), [2 M_est]);
        if is_plot
            scatter(e_est(1,:), e_est(2,:), 'g.');
            % Plot 95%-confidence ellipses around landmarks
            for i = 1:M
                plot_ellipse(sol.history(T).P((2*i-1+offset):(2*i+offset),(2*i-1+offset):(2*i+offset)) * 5.991, e_est(:,i), 'g');
            end
        end
    end
    
    if compute_errors
        % Compute errors between estimate, ground truth, and toolbox EKF estimate
        % For each estimate, construct 2*M_truth matrix of landmark positions,
        % where M_truth is the true number of landmarks (in argument map);
        % undetected landmarks will have NaN values in their estimates
        map_t = map.map;
        map_x = NaN(size(map_t));
        for i = 1:length(indices)
            map_x(:,indices(i)) = x_est{T}((2*i-1+offset):(2*i+offset));
        end
        if ekf_exists
            map_e = NaN(size(map_t));
            for i = 1:size(map_t, 2)
                if ~isnan(sol.landmarks(1,i))
                    map_e(:,i) = sol.history(T).x_est((sol.landmarks(1,i)+offset):(sol.landmarks(1,i)+1+offset));
                end
            end
        end

        % Only compute errors between landmarks that are estimated by both
        % Return number of landmarks that are estimated by both,
        % as well as average error in the overlapping landmarks
        mask_xt = ~isnan(map_x) & ~isnan(map_t);
        num_xt = sum(sum(mask_xt)) / 2;  
        err_xt = average_error(reshape(map_x(mask_xt), [2 num_xt]), reshape(map_t(mask_xt), [2 num_xt]));
        if ekf_exists
            mask_et = ~isnan(map_e) & ~isnan(map_t);
            num_et = sum(sum(mask_et)) / 2;
            err_et = average_error(reshape(map_e(mask_et), [2 num_et]), reshape(map_t(mask_et), [2 num_et]));
            mask_xe = ~isnan(map_x) & ~isnan(map_e);
            num_xe = sum(sum(mask_xe)) / 2;
            err_xe = average_error(reshape(map_x(mask_xe), [2 num_xe]), reshape(map_e(mask_xe), [2 num_xe]));
        else
            num_et = NaN;
            err_et = NaN;
            num_xe = NaN;
            err_xe = NaN;
        end
    else
        num_xt = NaN;
        err_xt = NaN;
        num_et = NaN;
        err_et = NaN;
        num_xe = NaN;
        err_xe = NaN;
    end
end