% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 10 that denotes question
%                       number to run.
%        efk_l, ekf_m, ekf_s -> Optional pre-computed Robotics toolbox EKF
%                               objects computed in question E0
% Output: efk_l, ekf_m, ekf_s -> If EKF objects were computed, you can save
%                               it and pass it in on later calls to
%                               ex4_slam to avoid re-computing it

function [ekf_l, ekf_m, ekf_s] = ex4_slam(questionNum, ekf_l, ekf_m, ekf_s)

    close all;
    
    if nargin < 1
        error('Error: Please enter a question number as a parameter');
    end
    
    % Certain questions may require changes to parameters in parameters.m
    [V, W, x0, P0, range, fov, T, num_trials] = parameters();
    
    % ========== Question E0 ==========
    if questionNum == 0
        % TODO: Use the Robotics toolbox to generate data and use the EKF
        % class to perform EKF-based localization, mapping, and SLAM
        [ekf_l, ekf_m, ekf_s] = E0(V, W, x0, P0, range, fov);
    end
    
    % ========== Question E1 ==========
    if questionNum == 1
        load('e1.mat');
        % TODO: Implement EKF-based localization (assuming known landmarks);
        % result should be similar to ekf_l
        [x_est, P_est] = E1(odo_l, zind_l, z_l, V, W, x0, P0, map_l);
        if nargin > 1
            visualize(x_est, P_est, [], map_l, veh_l, 'l', ekf_l, true);
        else
            visualize(x_est, P_est, [], map_l, veh_l, 'l', [], true);
        end
    end

    % ========== Question E2 ==========
    if questionNum == 2
        load('e2.mat');
        % TODO: Implement EKF-based mapping (assuming known robot trajectory);
        % result should be similar to ekf_m
        [x_est, P_est, indices] = E2(odo_m, zind_m, z_m, W, x0);
        if nargin > 2
            visualize(x_est, P_est, indices, map_m, veh_m, 'm', ekf_m, true);
        else
            visualize(x_est, P_est, indices, map_m, veh_m, 'm', [], true);
        end
    end
    
    % ========== Question E3 ==========
    if questionNum == 3
        load('e3.mat');
        % TODO: Implement EKF-based SLAM
        [x_est, P_est, indices] = E3(odo_s, zind_s, z_s, V, W, x0, P0);
        if nargin > 3
            visualize(x_est, P_est, indices, map_s, veh_s, 's', ekf_s, true);
        else
            visualize(x_est, P_est, indices, map_s, veh_s, 's', [], true);
        end
    end
    
    % ========== Question E4 ==========
    if questionNum == 4
        load('e3_r8.mat');
        range = 8;
        % TODO: Use the toolbox EKF to perform SLAM with maximum range of 8
        ekf_s_R8 = E4(V, W, x0, P0, range, fov);
        [x_est, P_est, indices] = E3(odo_s, zind_s, z_s, V, W, x0, P0);
        visualize(x_est, P_est, indices, map_s, veh_s, 's', ekf_s_R8, true);
    end

    % ========== Question E5 ==========
    if questionNum == 5
        load('e2.mat');
        % TODO: Implement a noisy sensor to generate noisy odometry readings
        % and noisy sensor measurements, given the true odometry readings
        % in odo_m and true landmark positions in map_m
        % -- odo_m and map_m come from E2, where the vehicle pose was known
        [odo, zind, z] = E5(odo_m, map_m, V, W, x0, range, fov, 'o');
        % E5 returns zind indices as a 1xT cell array,
        % but E3 expects zind to be a 1xT vector;
        % code below assumes zind{t} is either empty or is a 1x1 vector
        zind_vector = zeros(1, T);
        for t = 1:length(zind)
            if ~isempty(zind{t})
                zind_vector(t) = zind{t};
            end
        end
        % Perform SLAM on the generated noisy data;
        % result should be similar to E3, but variations are expected
        % since the generated noisy data is different every time
        [x_est, P_est, indices] = E3(odo, zind_vector, z, V, W, x0, P0);
        visualize(x_est, P_est, indices, map_m, veh_m, 's', [], true);
    end
    
    % ========== Question E6 ==========
    if questionNum == 6
        load('e2.mat');
        % TODO: Design trajectories that lead to better SLAM performance
        [x_truth, odo_truth] = E6(x0, T);
        if max(odo_truth(1,:)) > 0.1
            disp('Error: Single-step translation exceeds 0.1');
        end
        if max(abs(odo_truth(2,:))) > 0.0546
            disp('Error: Single-step rotation exceeds 0.0546');
        end
        % To use data from e2.mat instead of designed trajectory in E6,
        % remove comments for the following two lines
        % x_truth = veh_m.x_hist';
        % odo_truth = odo_m;
        
        % Since noisy data generated from E5 is random, multiple trials
        % are needed to assess overall performance (>= 100 recommended).
        % For debugging purposes, num_trials is 1 by default.
        statistics = [];
        for trial = 1:num_trials
            % Generate noisy data while following custom trajectory from E6
            [odo, zind, z] = E5(odo_truth, map_m, V, W, x0, range, fov, 'o');
            % E5 returns zind indices as a 1xT cell array,
            % but E3 expects zind to be a 1xT vector;
            % code below assumes zind{t} is either empty or is a 1x1 vector
            zind_vector = zeros(1, T);
            for t = 1:length(zind)
                if ~isempty(zind{t})
                    zind_vector(t) = zind{t};
                end
            end
            % Perform SLAM on the generated noisy data
            [x_est, P_est, indices] = E3(odo, zind_vector, z, V, W, x0, P0);
            veh_m.x_hist = x_truth';
            if num_trials > 1
                statistics = [statistics; visualize(x_est, P_est, indices, map_m, veh_m, 'h', [], true)];
            end
        end
        
        % If num_trials == 1, then the result is visualized;
        % otherwise, there is no visualization, instead error statistics
        % are aggregated and shown as a collection of histograms.
        % For debugging purposes, num_trials is 1 by default.
        if num_trials == 1
            visualize(x_est, P_est, indices, map_m, veh_m, 's', [], length(x_est) == T);
        else
            visualize_histogram(statistics);
        end
    end
    
    % ========== Question E7 ==========
    if questionNum == 7
        load('e2.mat');
        % Since noisy data generated from E5 is random, multiple trials
        % are needed to assess overall performance (>= 100 recommended).
        % For debugging purposes, num_trials is 1 by default.
        statistics = [];
        for trial = 1:num_trials
            % TODO: Generate noisy data while following trajectory in odo_m;
            % the 'a' mode of E5 generates observations from all visible
            % landmarks at each time step, not just one at random
            [odo, zind, z] = E5(odo_m, map_m, V, W, x0, range, fov, 'a');
            % TODO: Perform SLAM on the generated noisy data,
            % where each time step can have multiple landmark observations
            [x_est, P_est, indices] = E7(odo, zind, z, V, W, x0, P0);
            if num_trials > 1
                statistics = [statistics; visualize(x_est, P_est, indices, map_m, veh_m, 'h', [], true)];
            end
        end
        
        % If num_trials == 1, then the result is visualized;
        % otherwise, there is no visualization, instead error statistics
        % are aggregated and shown as a collection of histograms.
        % For debugging purposes, num_trials is 1 by default.
        if num_trials == 1
            visualize(x_est, P_est, indices, map_m, veh_m, 's', [], length(x_est) == T);
        else
            visualize_histogram(statistics);
        end
    end
    
    % ========== Question E8 ==========
    if questionNum == 8
        load('e2.mat');
        % Since noisy data generated from E5 is random, multiple trials
        % are needed to assess overall performance (>= 100 recommended).
        % For debugging purposes, num_trials is 1 by default.
        statistics = [];
        for trial = 1:num_trials
            % Generate noisy data while following trajectory in odo_m;
            % the 'a' mode of E5 generates observations from all visible
            % landmarks at each time step, not just one at random
            [odo, zind, z] = E5(odo_m, map_m, V, W, x0, range, fov, 'a');
            % For bearing-only SLAM, all range measurements are removed
            z_b = cell(size(z));
            for t = 1:length(z)
                if ~isempty(z{t})
                    z_b{t} = z{t}(2,:);
                end
            end
            % TODO: Implement bearing-only SLAM
            [x_est, P_est, indices] = E8(odo, zind, z_b, V, W(2,2), range, fov, x0, P0);
            if num_trials > 1
                statistics = [statistics; visualize(x_est, P_est, indices, map_m, veh_m, 'h', [], true)];
            end
        end
        
        % If num_trials == 1, then the result is visualized;
        % otherwise, there is no visualization, instead error statistics
        % are aggregated and shown as a collection of histograms.
        % For debugging purposes, num_trials is 1 by default.
        if num_trials == 1
            visualize(x_est, P_est, indices, map_m, veh_m, 's', [], length(x_est) == T);
        else
            visualize_histogram(statistics);
        end
    end
    
    % ========== Question E9 ==========
    if questionNum == 9
        load('e2.mat');
        % Since noisy data generated from E5 is random, multiple trials
        % are needed to assess overall performance (>= 100 recommended).
        % For debugging purposes, num_trials is 1 by default.
        statistics = [];
        for trial = 1:num_trials
            % Generate noisy data while following trajectory in odo_m;
            % the 'a' mode of E5 generates observations from all visible
            % landmarks at each time step, not just one at random
            [odo, zind, z] = E5(odo_m, map_m, V, W, x0, range, fov, 'a');
            % For range-only SLAM, all bearing measurements are removed
            z_r = cell(size(z));
            for t = 1:length(z)
                if ~isempty(z{t})
                    z_r{t} = z{t}(1,:);
                end
            end
            % TODO: Implement range-only SLAM
            [x_est, P_est, indices] = E9(odo, zind, z_r, V, W(1,1), range, fov, x0, P0);
            if num_trials > 1
                statistics = [statistics; visualize(x_est, P_est, indices, map_m, veh_m, 'h', [], true)];
            end
        end
        
        % If num_trials == 1, then the result is visualized;
        % otherwise, there is no visualization, instead error statistics
        % are aggregated and shown as a collection of histograms.
        % For debugging purposes, num_trials is 1 by default.
        if num_trials == 1
            visualize(x_est, P_est, indices, map_m, veh_m, 's', [], length(x_est) == T);
        else
            visualize_histogram(statistics);
        end
    end
    
    % ========== Question E10 ==========
    if questionNum == 10
        load('e2.mat');
        % Since noisy data generated from E5 is random, multiple trials
        % are needed to assess overall performance (>= 100 recommended).
        % For debugging purposes, num_trials is 1 by default.
        statistics = [];
        for trial = 1:num_trials
            % TODO: Generate noisy data while following trajectory in odo_m;
            % the 'f' mode of E5 generates observations from all visible
            % landmarks at each time step, but occasionally changes
            % measurements into uniformly distributed false positives
            [odo, zind, z] = E5(odo_m, map_m, V, W, x0, range, fov, 'f');
            % Perform SLAM on the generated noisy data,
            % where each time step can have multiple landmark observations
            [x_est, P_est, indices] = E7(odo, zind, z, V, W, x0, P0);
            if num_trials > 1
                statistics = [statistics; visualize(x_est, P_est, indices, map_m, veh_m, 'h', [], true)];
            end
        end
        
        % If num_trials == 1, then the result is visualized;
        % otherwise, there is no visualization, instead error statistics
        % are aggregated and shown as a collection of histograms.
        % For debugging purposes, num_trials is 1 by default.
        if num_trials == 1
            visualize(x_est, P_est, indices, map_m, veh_m, 's', [], length(x_est) == T);
        else
            visualize_histogram(statistics);
        end
    end

end