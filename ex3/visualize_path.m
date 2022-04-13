% Visualizes the estimated landmarks, ground truth,
% and optional toolbox EKF estimate (computed only if sol passed in).
% Returns error statistics between each pair of comparisons.

% Input: x_est -> 1xT cell array containing the map state mean
%                 for T time steps (i.e., x_est{t} is a (3+2M)x1 vector,
%                 where M is the number of landmarks observed by time t)
%        P_est -> 1xT cell array containing the vehicle state covariance
%                 for T time steps (i.e., P_est{t} is a (3+2M)x(3+2M) matrix,
%                 where M is the number of landmarks observed by time t)
%        veh -> Robotics toolbox Vehicle object containing the
%               known vehicle (known trajectory)
%        sol -> Robotics toolbox EKF object containing the toolbox estimate;
%               if passed in, the estimate will be visualized and errors
%               will be computed; if no solution, pass in empty matrix []
%        is_plot -> Boolean value; map is visualized if and only if true
%        compute_errors -> Boolean value; if true, errors are computed;
%                          if false, estimation errors are not computed
% Output: [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe];
%         There are 2 triples of errors, for trajecotry position error
%         and trajectory angular error respectively.
%         Each triple is a pairwise comparison between estimate and truth,
%         toolbox EKF and truth, estimate and toolbox EKF respectively.

function [err_xt, err_et, err_xe, ang_xt, ang_et, ang_xe] = visualize_path(x_est, P_est, veh, sol, is_plot, compute_errors)

    % Collect the estimated vehicle trajectory
    T = length(x_est);
    v_est = zeros(T,3);
    for t = 1:T
        v_est(t,:) = x_est{t}(1:3);
    end
    
    if is_plot
        % Plot the estimated vehicle trajectory as a red line
        plot(v_est(:,1), v_est(:,2), 'r');
        % Plot 95%-confidence ellipses at every 10th state in trajectory
        for t = 1:10:T
            plot_ellipse(P_est{t}(1:2,1:2) * 5.991, v_est(t,1:2), 'r');
        end
    end
    
    % If toolbox EKF object is passed in, visualize the estimate too
    ekf_exists = ~isempty(sol);
    if ekf_exists
        % Collect and plot the estimated vehicle trajectory in green
        e_est = sol.get_xy();
        if is_plot
            plot(e_est(:,1), e_est(:,2), 'g');
            % Plot 95%-confidence ellipses at every 10th state in trajectory
            for t = 10:10:T
                plot_ellipse(sol.history(t).P(1:2,1:2) * 5.991, e_est(t,1:2), 'g');
            end
        end
    end

    if compute_errors
        % Compute errors between estimate, ground truth, and toolbox EKF estimate
        err_xt = average_error(v_est(:,1:2)', veh.x_hist(:,1:2)');
        ang_xt = mean(abs(angdiff(v_est(:,3), veh.x_hist(:,3))));
        if ekf_exists
            err_et = average_error(e_est(:,1:2)', veh.x_hist(:,1:2)');
            ang_et = mean(abs(angdiff(e_est(:,3), veh.x_hist(:,3))));
            err_xe = average_error(v_est(:,1:2)', e_est(:,1:2)');
            ang_xe = mean(abs(angdiff(v_est(:,3), e_est(:,3))));
        else
            err_et = NaN;
            err_xe = NaN;
            ang_et = NaN;
            ang_xe = NaN;
        end
    else
        err_xt = NaN;
        err_et = NaN;
        err_xe = NaN;
        ang_xt = NaN;
        ang_et = NaN;
        ang_xe = NaN;
    end
end