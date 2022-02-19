% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    % config parameters.
    step_size = 0.05; freq_sample_goal = 0.5; tolerance = 0.1; max_nodes = 800;
    % we will refer to nodes with an index, which refers to an actual
    % configuration in the Nx4 `vertices` matrix.
    num_nodes = 1;
    vertices = q_start;
    % create the tree as a graph.
    tree = graph([1],[]);

    % init in case we fail to find a path before hitting max_nodes.
    path = []; path_found = false;

    % continue adding random nodes to the tree until we hit the goal or the
    % max allowable number of nodes.
    while num_nodes < max_nodes
        if rand(1) <= freq_sample_goal
            % sample the goal directly.
            q_target = q_goal;
        else
            % sample a random configuration in bounds.
            q_target = rand(1, 4) .* (q_max - q_min) + q_min;
        end
        % find nearest node to this point.
        q_near_i = 0; min_dist = 100;
        for i = 1:num_nodes
            dist_to_target = norm(vertices(i,:)-q_target);
            if dist_to_target < min_dist
                min_dist = dist_to_target;
                q_near_i = i;
            end
        end
        q_near = vertices(q_near_i,:);
        % get the configuration `step_size` distance along the unit vector
        % from q_near to q_target.
        q_new = q_near + step_size * ((q_target - q_near)/min_dist);
        % check for collisions from q_near to q_new.
        if ~check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)
%             disp no_collision
            % add q_new to our tree.
            vertices = [vertices; q_new];
            num_nodes = num_nodes + 1;
            tree = addnode(tree,1);
            tree = addedge(tree,[q_near_i],[num_nodes]);
        end

        % check if we have just added the goal to the tree, and finish if so.
        if norm(vertices(num_nodes,:) - q_goal) < tolerance
            disp FOUND_GOAL
            % we know there is exactly 1 path because it's a tree.
            path_found = true;
            node_path = shortestpath(tree,1,num_nodes);
            for p = 1:size(node_path,2)
                path = [path; vertices(node_path(1,p),:)];
            end
            % explicitly ensure path ends at goal config.
            path = [path; q_goal];
            return
        end
    end
    disp FAILED_TO_FIND_PATH
end

%     figure;
%     plot(tree)