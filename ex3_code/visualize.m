% Visualizes the estimated trajectory and landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).

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
%        veh -> Robotics toolbox Vehicle object containing the
%               known vehicle (known trajectory)
%        mode -> 'l' for localization, 'm' for mapping, 's' for SLAM,
%                'h' for statistics
%                In 'l', 'm', 's' modes, the ground truth and estimated
%                trajectory and/or map is visualized, and estimation errors
%                are printed out if compute_errors is true
%                In 'h' mode, visualization and printing is suppressed,
%                since many trials are assumed.
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
%                          (unless in 'h' mode); this allows partial
%                          trajectories and maps to be visualized as well.
% Output: statistics -> 1x12 vector of error statistics
%         = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe,
%            err_xtm, err_etm, err_xem, num_xt, num_et, num_xe]
%         There are 4 triples of errors, for trajecotry position error,
%         trajectory angular error, landmark position error, and
%         number of landmarks detected respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function statistics = visualize(x_est, P_est, indices, map, veh, mode, sol, compute_errors)

    statistics = NaN(1, 12);

    % Plot ground truth landmarks (black hexagrams) and trajectory (blue line)
    if mode == 'l' || mode == 'm' || mode == 's' || mode == 'n'
        figure;
        scatter(map.map(1,:), map.map(2,:), 'kh');
        hold on;
        plot(veh.x_hist(:,1), veh.x_hist(:,2), 'b');
        sol_exists = ~isempty(sol);
    end

    % Visualize the estimated trajectory
    if mode == 'l' || mode == 's'
        if sol_exists
            [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh, sol, true, compute_errors);
        else
            [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh, [], true, compute_errors);
        end
        statistics(1, 1:6) = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe];
        
        if compute_errors
            T = length(x_est);
            fprintf('Trajectory errors (average Euclidean and angular distances over %d timesteps):\n', T);
            fprintf('-- between estimate and ground truth: \n   Pos %f \tOri %f\n', err_xtv, ang_xt);
            if sol_exists
                fprintf('-- between toolbox EKF estimate and ground truth: \n   Pos %f \tOri %f\n', err_etv, ang_et);
                fprintf('-- between estimate and toolbox EKF estimate: \n   Pos %f \tOri %f\n', err_xev, ang_xe);
            end
        end
    end
    
    % Visualize the estimated landmarks
    if mode == 'm' || mode == 's'
        if sol_exists
            [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, mode == 's', sol, true, compute_errors);
        else
            [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, mode == 's', [], true, compute_errors);
        end
        statistics(1, 7:12) = [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe];
        
        if compute_errors
            fprintf('Landmark errors (average Euclidean distance for landmarks estimated by both):\n');
            fprintf('-- between estimate and ground truth (%d landmarks): \n   %f\n', num_xt, err_xtm);
            if sol_exists
                fprintf('-- between toolbox EKF estimate and ground truth (%d landmarks): \n   %f\n', num_et, err_etm);
                fprintf('-- between estimate and toolbox EKF estimate (%d landmarks): \n   %f\n', num_xe, err_xem);
            end
        end
    end
    
    % Get statistics only (no visualization)
    if mode == 'h'
        [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh, [], false, compute_errors);
        [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe] = visualize_map(x_est, P_est, indices, map, true, [], false, compute_errors);
        statistics(1, 1:6) = [err_xtv, err_etv, err_xev, ang_xt, ang_et, ang_xe];
        statistics(1, 7:12) = [err_xtm, err_etm, err_xem, num_xt, num_et, num_xe];
    end
end