% Input: x0 -> 3x1 vector denoting the initial vehicle state
%        T -> Scalar denoting the number of time steps in the trajectory
% Output: x_truth -> 3xT matrix containing true (non-noisy) trajectory
%         odo_truth -> 2xT matrix containing true (non-noisy)
%                      odometry readings for T time steps

function [x_truth, odo_truth] = E6(x0, T)
    % the map is not passed and we obviously need it,
    % so assume the default map.
    rng(0); map = LandmarkMap(20);
    % instantiate the trajectory.
    x_truth = zeros(3,T); odo_truth = zeros(2,T);
    % first pass attempt: just design a trajectory to pass through all
    % landmarks. treat as travelling salesman problem.
    % choose nearest landmark to initial pose as 1st node.
    cur_goal = 1; cur_dist = norm(map.landmark(cur_goal) - x0(1:2));
    for i_lm = 1:map.nlandmarks
        if norm(map.landmark(i_lm) - x0(1:2)) < cur_dist
            cur_goal = i_lm;
            cur_dist = norm(map.landmark(i_lm) - x0(1:2));
        end
    end
    % build directed graph using nearest neighbors approach.
    G = digraph();
    cur_node = cur_goal; % start where we decided above.
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
        G = addedge(G, cur_node, cur_goal);
        % mark node as visited and update current to there.
        cur_node = cur_goal;
        unvisited = unvisited(unvisited~=cur_node);
        cur_goal = 0; cur_dist = -1;
    end
    plot(G) %DEBUG
    % traverse our graph to generate actual trajectory.

%     for t = 1:T
%         
%     end
end