% DO NOT MODIFY THIS FILE!
% Input: questionNum -> Integer between 0 and 4 that denotes question
%                       number to run.
%        efk_l, ekf_m, ekf_s -> Optional pre-computed Robotics toolbox EKF
%                               objects computed in question E0
% Output: efk_l, ekf_m, ekf_s -> If EKF objects were computed, you can save
%                               it and pass it in on later calls to ex3_ekf
%                               to avoid re-computing it

function [ekf_l, ekf_m, ekf_s] = ex3_ekf(questionNum, ekf_l, ekf_m, ekf_s)

    close all;
    
    if nargin < 1
        error('Error: Please enter a question number as a parameter');
    end
    
    % Certain questions may require changes to parameters in parameters.m
    [V, W, x0, P0, range, fov] = parameters();
    
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

end