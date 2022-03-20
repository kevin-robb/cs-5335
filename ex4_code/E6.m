% Input: x0 -> 3x1 vector denoting the initial vehicle state
%        T -> Scalar denoting the number of time steps in the trajectory
% Output: x_truth -> 3xT matrix containing true (non-noisy) trajectory
%         odo_truth -> 2xT matrix containing true (non-noisy)
%                      odometry readings for T time steps

function [x_truth, odo_truth] = E6(x0, T)
    % odom constraints:
    odom_d_max = 0.1; odom_th_max = 0.0546;
    % the map is not passed and we obviously need it, so assume the default map.
    rng(0); map = LandmarkMap(20);
    % instantiate the trajectory.
    x_truth = zeros(3,T); odo_truth = zeros(2,T);

    % set which trajectory will be used. 1=TSP, 2=manual.
    TRAJ_CHOICE = 1;

    lm_path = [];
    if TRAJ_CHOICE == 1
        % for any map, we can design a trajectory to pass through all landmarks,
        % by treating it as travelling salesman problem, and use NN heuristic.
        % choose nearest landmark to initial pose as 1st node.
        cur_goal = 1; cur_dist = norm(map.landmark(cur_goal) - x0(1:2));
        for i_lm = 1:map.nlandmarks
            if norm(map.landmark(i_lm) - x0(1:2)) < cur_dist
                cur_goal = i_lm;
                cur_dist = norm(map.landmark(i_lm) - x0(1:2));
            end
        end
    
        % build directed graph using nearest neighbors approach.
        cur_node = cur_goal; % start where we decided above.
        % store path of landmark indexes to visit in order.
        lm_path = [cur_node]; % track path as a 1xN vector.
        unvisited = 1:map.nlandmarks; 
        unvisited = unvisited(unvisited~=cur_node);
        cur_goal = 0; cur_dist = -1;
        while ~isempty(unvisited)
            % find nearest neighbor.
            for i_lm = unvisited
                new_dist = norm(map.landmark(i_lm) - map.landmark(cur_node));
                if cur_dist < 0 || new_dist < cur_dist
                    cur_goal = i_lm;
                    cur_dist = new_dist;
                end
            end
            % add this edge.
            lm_path = [lm_path, cur_goal];
            % mark node as visited and update current.
            cur_node = cur_goal;
            unvisited = unvisited(unvisited~=cur_node);
            cur_goal = 0; cur_dist = -1;
        end
    elseif TRAJ_CHOICE == 2
        % manually choose waypoints for this map such that the veh will
        % pass through only high density areas.
        lm_path = [14,19,16,8,7,12,1];
    end
    % traverse our graph to generate an actual trajectory.
    t = 1; x_v = x0;
    while t <= T
        % first entry in lm_path is always the goal.
        % if we're pretty close to our current goal, remove it
        % to mark it done, and add it to the end so we can loop
        % around if time allows.
        if norm(x_v(1:2) - map.landmark(lm_path(1))) < 1.7
            lm_path = [lm_path, lm_path(1)];
            lm_path(1) = [];
        else % move towards current goal.
            % compute vector from veh position to the landmark.
            diff_vec = map.landmark(lm_path(1)) - x_v(1:2);
            % extract range and bearing.
            r = norm(diff_vec); % range.
            b = atan2(diff_vec(2), diff_vec(1)); % global bearing.
            beta = angdiff(b - x_v(3)); % bearing relative to robot.
            % choose odom cmd based on constraints.
            r = min(r, odom_d_max); % always positive.
            if abs(beta) > odom_th_max
                % cap angle magnitude but keep sign.
                beta = odom_th_max * sign(beta);
            end
            odom = [r; beta];
            % update vehicle position given this odom.
            x_v = [x_v(1) + r*cos(x_v(3));
                   x_v(2) + r*sin(x_v(3));
                   x_v(3) + beta];
            % add these to the trajectory.
            x_truth(:,t) = x_v;
            odo_truth(:,t) = odom;
            t = t+1;
        end
    end
end





















