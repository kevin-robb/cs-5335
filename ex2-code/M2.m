% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    % generate vertices via unif rand sampling.
    % same as M1 but check for collisions.
    range = q_max - q_min;
    samples = [];
    while size(samples,1) < num_samples
        % generate a random sample in bounds, and add it if no collision.
        new_vert = rand(1, 4) .* range + q_min;
        if ~check_collision(robot, new_vert, link_radius, sphere_centers, sphere_radii)
            samples = [samples; new_vert];
        end
    end
    % create a matrix with the distances between all pairs.
    distances = zeros(num_samples,num_samples);
    % note: this could be more efficient by taking advantage of symmetry,
    % but this is just easier.
    for i = 1:num_samples
        for j = 1:num_samples 
            if i == j
                distances(i,j) = NaN;
            else
                distances(i,j) = norm(samples(j,:)-samples(i,:));
            end
        end
    end
    % now we need to connect vertices with edges. each point will be
    % connected to its nearest `num_neighbors` neighbors if
    % collision-free paths exist.
    adjacency = zeros(num_samples,num_samples);
    for i = 1:num_samples 
        nbrs = distances(i,:);
        for n = 1:num_neighbors
            % find nearest neighbor.
            [dist,j] = min(nbrs,[],'omitnan');
            % prevent this neighbor from being chosen again.
            nbrs(j) = NaN;
            % check this path for no collisions.
            if ~check_edge(robot, samples(i,:), samples(j,:), link_radius, sphere_centers, sphere_radii)
                % add this entry to the adjacency map.
                adjacency(i,j) = dist;
                adjacency(j,i) = dist;
            end
        end
    end
end












