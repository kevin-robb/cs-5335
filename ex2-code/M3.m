% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    % we first need to create a graph from our samples and adjacency map.
    s = []; t = []; weights = [];
    for i = 1:(size(samples,1)-1)
        for j = (i+1):size(samples,1)
            % only make an edge if they have a distance saved.
            if adjacency(i,j) ~= 0
                s = [s, i];
                t = [t, j];
                weights = [weights, adjacency(i,j)];
            end
        end
    end
    G = graph(s,t,weights);
%     figure;
%     plot(G,'EdgeLabel',G.Edges.Weight)

    % get the distance from each node to the start and goal.
    dist_to_start = [];
    dist_to_goal = [];
    for i = 1:size(samples,1)
        dist_to_start = [dist_to_start; norm(samples(i,:)-q_start)];
        dist_to_goal = [dist_to_goal; norm(samples(i,:)-q_goal)];
    end
    % find the closest node w/o collision to each.
    node_start = 0; node_goal = 0;
    while node_start == 0
        [~,j] = min(dist_to_start,[],'omitnan');
        dist_to_start(j) = NaN;
        if ~check_edge(robot, q_start, samples(j,:), link_radius, sphere_centers, sphere_radii)
            node_start = j;
        end
    end
    while node_goal == 0
        [~,j] = min(dist_to_goal,[],'omitnan');
        dist_to_goal(j) = NaN;
        if ~check_edge(robot, samples(j,:), q_goal, link_radius, sphere_centers, sphere_radii)
            node_goal = j;
        end
    end
    if node_start == 0 || node_goal == 0
        % no path exists to the PRM from either the start or goal config.
        path_found = false;
        path = [];
        return
    end

    % find the shortest path on the graph between these nodes.
    node_path = shortestpath(G,node_start,node_goal);
    % if the path is empty, no path exists.
    if isempty(node_path)
        path_found = false;
        path = [];
        return
    end

    % getting here means a path does exist.
    path_found = true;
    % convert the path from nodes to configurations.
    path = q_start;
    for p = 1:size(node_path)
        path = [path; samples(node_path(p),:)];
    end
    path = [path; q_goal];
end





