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
    smoothed_path = path(1,:);
    i = 1; % can't use for loop since I want to change the index sometimes.
    while i < size(path,1)
        temp_i = i;
        % add link i to the path. it's necessary if we get here.
        smoothed_path = [smoothed_path; path(i,:)];
        % try to find a connection to a point as far away as possible.
        for j = size(path,1):-1:(i+1)
            if ~check_edge(robot, path(i,:), path(j,:), link_radius, sphere_centers, sphere_radii)
                disp('found shortcut from i to j');
                i
                j
                % a path exists, update i=j to skip all intermediary points.
                i = j;
                break
            end
        end
        % advance the index if nothing was found
        if i == temp_i
            i = i + 1;
        end
        % add this node to the new path.
        smoothed_path = [smoothed_path; path(i,:)];
    end
end