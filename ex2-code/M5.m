% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    % go front to back and try to skip as many unnecessary waypoints as
    % possible and build a more efficient path.
    smoothed_path = [];
    i = 1; % can't use for loop since I want to change the index sometimes.
    while i < size(path,1)
        i = i + 1;
        % add link i to the path. it's necessary if we get here.
        smoothed_path = [smoothed_path; path(i,:)];
        % try to find a connection to a point as far away as possible.
        for j = size(path,1):-1:(i+1)
            if ~check_edge(robot, path(i,:), path(j,:), link_radius, sphere_centers, sphere_radii)
                disp('found shortcut from i to j');
                i
                j
                % a path exists, so add j directly to the list.
                smoothed_path = [smoothed_path; path(j,:)];
                % update i=j to skip all intermediary points
                i = j;
                break
            end
        end
    end
end