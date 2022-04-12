% Code for V3, RANSAC implementation for detecting geometries in scene.
close all; clear all;
%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('ptcloud.mat'); % two MxNx3 pcs called "ptcloud_rgb" and "ptcloud_xyz".
% display whole pointcloud.
% show_pointcloud(ptcloud_xyz, ptcloud_rgb)
cloud = ptcloud_xyz;

% compute surface norms. (~15 seconds)
USE_PRECOMPUTED_NORMALS = true;
normals = compute_all_normals(cloud, USE_PRECOMPUTED_NORMALS);

%%%%%%%%%%%%%%%%%%%%%%%%% PLANES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find all planes with RANSAC. (~30 seconds)
% Optional third arg allows execution to stop early when all the 
% planes have been found. Leave out to run all iterations.
[planes, cloud] = ransac_planes(cloud, normals, 3);
show_planes(planes);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% SPHERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find the sphere's params with RANSAC. (~1 second)
[center, radius, cloud] = ransac_sphere(cloud, normals);
show_sphere(cloud, radius, center);

%%%%%%%%%%%%%%%%%%%%%%%%%% CYLINDER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find the cylinder's params with RANSAC. (~45-90 seconds)
[center, radius, length, axis] = ransac_cylinder(cloud, normals);
show_cylinder(cloud, center, radius, length, axis)



%%%%%%%%%%%%%%%%%% COMPUTATIONAL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to perform RANSAC to find the cylinder in a scene.
% @param cloud: MxNx3 array of points.
% @param normals: MxNx3 set of surface normal vectors for pts in cloud.
% @return center: 3x1 point of center of cylinder.
% @return radius: float. radius of cylinder.
% @return length: float. distance between circular faces.
% @return axis: 3x1 unit vector describing orientation of axis.
function [center, radius, length, axis] = ransac_cylinder(cloud, normals)
    M = size(cloud, 1); N = size(cloud, 2);
    RADIUS_RANGE = [0.05, 0.10]; % meters.
    NUM_RANSAC_TRIALS = 1000; trial_num = 0;
    MIN_PTS_ON_CYL = 8000; 
    EPSILON_RAD = 0.02; EPSILON_ANG = 0.1;
    REGION_SIZE = 200; % radius of area in pixels to search around candidate pt.
    while trial_num < NUM_RANSAC_TRIALS
        trial_num = trial_num + 1
        % choose two points at random, and assume they lie on the cylinder.
        % first choose one point at random.
        while 1
            % choose random indexes. ensure it has a normal vector.
            % (pts with any NaN have already been given 0 for norm vec.)
            r1 = randi([1,M],1); c1 = randi([1,N],1);
            if ~all(normals(r1,c1,:) == 0) && ~any(isnan(cloud(r1,c1,:)))
                p1 = [cloud(r1,c1,1); cloud(r1,c1,2); cloud(r1,c1,3)];
                n1 = [normals(r1,c1,1); normals(r1,c1,2); normals(r1,c1,3)];
                break
            end
        end
        % choose a second point at random that is nearby.
        r_min = max(1, r1 - REGION_SIZE); r_max = min(M, r1 + REGION_SIZE);
        c_min = max(1, c1 - REGION_SIZE); c_max = min(N, c1 + REGION_SIZE);
        while 1
            % choose random indexes. ensure it has a normal vector.
            r2 = randi([r_min,r_max],1); c2 = randi([c_min,c_max],1);
            if ~all(normals(r2,c2,:) == 0) && ~any(isnan(cloud(r2,c2,:)))
                p2 = [cloud(r2,c2,1); cloud(r2,c2,2); cloud(r2,c2,3)];
                n2 = [normals(r2,c2,1); normals(r2,c2,2); normals(r2,c2,3)];
                % make sure pts' surface norms aren't parallel.
                if abs(dot(n1, n2)) < 1 %- EPSILON_ANG / 10
                    break
                end
            end
        end
        p1
        p2
%         % find the implied axis direction and a pt on that axis.
%         [ctr, axis, rad] = estimate_cylinder(p1, p2, n1, n2);
%         % ensure radius is within range.
%         if rad < RADIUS_RANGE(1) || rad > RADIUS_RANGE(2)
%             continue
%         end
        % ----------------------------------
        % Do it randomly instead of using my hard earned geometries.
        % use surface norms of pts to find central axis unit vector.
        axis = cross(n1, n2); axis = axis / norm(axis);
        % sample a radius at random in the range.
        rad = (RADIUS_RANGE(2)-RADIUS_RANGE(1)) * rand() + RADIUS_RANGE(1);
        % use pt's normal vector and radius to find center.
        ctr = p1 - n1 * rad;
        % ----------------------------------

        % dist of a pt from the axis line is:
        dist_to_axis = @(p) norm(cross(p-ctr, p-ctr-axis)); %/norm(a)
        % pt on axis nearest to p is:
        nearest_on_axis = @(p) ctr - dot(ctr-p,axis)*axis; %/norm(a)^2
    
        % track extreme pts on axis, keyed by sign diff.
        extreme_axis_pts = containers.Map;
        % compute distance of all nearby pts to this line,
        % and check their norm vec is orthogonal to axis.
        pts_on_cyl = []; ind_on_cyl = [];
        for i = r_min:r_max
        for j = c_min:c_max
            p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            % find point on axis at min dist from p_c.
            p_axis = nearest_on_axis(p_c);
            if all(normals(i,j,:) == 0)
                % pt either has NaNs or isn't part of a surface. skip it.
                continue
            % candidate pt's dist from ctr must be appx 'rad'.
            elseif abs(dist_to_axis(p_c) - rad) < EPSILON_RAD
                % normal vector must be appx parallel to vec to axis.
                n_c = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
                if 1 - abs(dot(n_c, (p_c-p_axis)/norm(p_c-p_axis))) < EPSILON_ANG
                    % candidate is an inlier.
                    ind_on_cyl = [ind_on_cyl, [i;j]];
                    pts_on_cyl = [pts_on_cyl, p_c];
                    % keep track of the axis pts on either end.
                    diff = p_axis - ctr;
                    sign_key = num2str(sign(diff(1)) * sign(diff(2)) * sign(diff(3)));
                    if ~isKey(extreme_axis_pts, sign_key)
                        % first pt in this direction.
                        extreme_axis_pts(sign_key) = p_axis;
                    elseif extreme_axis_pts(sign_key)
                        % not the first in this direction. save farthest.
                        if norm(extreme_axis_pts(sign_key) - ctr) < norm(p_axis - ctr)
                            extreme_axis_pts(sign_key) = p_axis;
                        end
                    end
                end
            end
        end
        end
        % if sufficiently many pts fit, call it good.
        if size(pts_on_cyl, 2) > MIN_PTS_ON_CYL
            disp(strcat("Found satisfactory cylinder on trial ", num2str(trial_num),"."))

            % compute additional parameters.
            radius = rad;
            % length = dist between the two extreme pts on axis.
            extreme_1 = extreme_axis_pts('-1');
            extreme_2 = extreme_axis_pts('1');
            length = norm(extreme_1 - extreme_2);
            % center point of cyl is their midpoint.
            center = (extreme_1 + extreme_2) / 2;

            % DEBUG show cylinder before snapping.
%             show_cylinder(cloud, center, radius, length, axis, 'Pre-snap cylinder')
            
            % snap to the best cylinder for this set of inliers.
%             [radius, axis] = snap_cylinder(cloud, normals, ind_on_cyl, center, radius, axis, RADIUS_RANGE);

            % show just the inliers.
            show_pointcloud(cloud); hold on
            show_inliers(pts_on_cyl)

            % save results and exit RANSAC loop.
            return
        end
    end
    disp(strcat("Failed to find cylinder in scene after ",num2str(NUM_RANSAC_TRIALS)," RANSAC iterations."))
end

% Function to snap cylinder params to max accuracy using only inliers.
function [radius, axis] = snap_cylinder(cloud, normals, inlier_inds, ctr, rad_est, axis_est, RADIUS_RANGE)
    M = size(inlier_inds, 2);
    disp(strcat("Number of inliers = ", num2str(M),"."))
    ITERATIONS = 200; iter_num = 0;
    best_error = Inf; best_rad = rad_est;
    best_axis = axis_est;
    while iter_num < ITERATIONS
        iter_num = iter_num + 1;
        % generate a random perturbation of the center, rad, and axis 
        % found by RANSAC from the full set of points.
        
        % sample a radius at random in the range.
%         rad = (RADIUS_RANGE(2)-RADIUS_RANGE(1)) * rand() + RADIUS_RANGE(1);
        % perturb axis with small random rotation.
        rx = SE3.Rx(0.5 * rand()); 
        ry = SE3.Ry(0.5 * rand());
        rz = SE3.Rz(0.5 * rand());
        perturbation = rx.R * ry.R * rz.R;
        axis = perturbation * best_axis;

        % dist of a pt from the axis line is:
        dist_to_axis = @(p) norm(cross(p-ctr, p-ctr-axis)); %/norm(a)
        % pt on axis nearest to p is:
        nearest_on_axis = @(p) ctr - dot(ctr-p,axis)*axis; %/norm(a)^2
    
        % compute distance of all nearby pts to this line,
        % and check their norm vec is orthogonal to axis.
        error = 0;
        for ind = 1:M
            i = inlier_inds(1,ind); j = inlier_inds(2,ind);
            p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            % find point on axis at min dist from p_c.
            p_axis = nearest_on_axis(p_c);
            % candidate pt's dist from ctr must be appx 'rad'.
            error_pos = abs(dist_to_axis(p_c) - best_rad);
            % normal vector must be appx parallel to vec to axis.
            n_c = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
            error_ang = 1 - abs(dot(n_c, (p_c-p_axis)/norm(p_c-p_axis)));
            error = error + error_pos; % + error_ang;
        end
        % check if this is the best cylinder so far.
        if error < best_error
            best_error = error;
%             best_rad = rad; 
            best_axis = axis;
        end
    end
    % return the best set of values found.
    radius = best_rad; axis = best_axis;
end

% Function to find a cylinder's central axis and a point on that axis.
% @param p1, p2: 3x1 XYZ coords of points on cylinder's lateral surface.
% @param n1, n2: 3x1 surface normal vectors at p1 and p2.
% @return ctr: 3x1 point on cylinder's central axis.
% @return axis: 3x1 unit vector, direction of axis.
% @return rad: float. radius implied by ctr and p1.
function [ctr, axis, rad] = estimate_cylinder(p1, p2, n1, n2)  
    % use surface norms of pts to find central axis unit vector.
    axis = cross(n1, n2); axis = axis / norm(axis);
    % project pts/vectors onto plane orthogonal to axis.
    % plane will include p1 and use axis as normal vector.
    % dist of a point x to the plane is:
    dist_to_plane = @(x) dot(axis, x) - dot(axis, p1);
    % project p2 onto plane.
    p2_proj = p2 - dist_to_plane(p2) * axis;
    % find projections of n1 and n2 on plane.
    n1_proj = n1 - dist_to_plane(p1 + n1) * axis;
    n2_proj = n2 - dist_to_plane(p2_proj + n2) * axis;
    % create coord system on plane.
    x_plane = n1_proj / norm(n1_proj);
    y_plane = cross(x_plane, axis);
    y_plane = y_plane / norm(y_plane);
    % make change of basis matrix.
    B = [x_plane'; y_plane'];
    % change all coords to plane frame.
    % pts relative to p1.
%     p1_plane = [0; 0];
    p2_plane = B * (p2_proj - p1);
    n1_plane = B * n1_proj;
    n2_plane = B * n2_proj;
    % compute intersection of lines in this frame.
    c_x = (- n2_plane(2)*n1_plane(1)*p2_plane(1) + n1_plane(1)*n2_plane(1)*p2_plane(2)) / (n1_plane(2)*n2_plane(1) - n2_plane(2)*n1_plane(1));
    c_y = n1_plane(2)/n1_plane(1)*c_x;
    % transform center pt back to 3d coords.
    ctr = B' * [c_x; c_y] + p1;
    % compute the implied radius of the cylinder.
    rad = norm(ctr - p1);
end

% Function to perform RANSAC to find the sphere in a scene.
% @param cloud: MxNx3 array of points.
% @param normals: MxNx3 set of surface normal vectors for pts in cloud.
% @return center, radius of sphere.
function [center, radius, cloud] = ransac_sphere(cloud, normals)
    M = size(cloud, 1); N = size(cloud, 2);
    RADIUS_RANGE = [0.05, 0.10]; % meters.
    NUM_RANSAC_TRIALS = 500; trial_num = 0;
    MIN_PTS_ON_SPHERE = 6000; 
    EPSILON_RAD = 0.015; EPSILON_ANG = 0.1;
    REGION_SIZE = 150; % radius of area in pixels to search around candidate pt.
    while trial_num < NUM_RANSAC_TRIALS
        trial_num = trial_num + 1;
        % choose a point at random, and assume it lies on the sphere.
        while 1
            % choose random indexes. ensure it has a normal vector.
            % (pts with any NaN have already been given 0 for norm vec.)
            r = randi([1,M],1); c = randi([1,N],1);
            if ~all(normals(r,c,:) == 0)
                pt = [cloud(r,c,1); cloud(r,c,2); cloud(r,c,3)];
                n_pt = [normals(r,c,1); normals(r,c,2); normals(r,c,3)];
                break
            end
        end
        % sample a radius at random in the range.
        rad = (RADIUS_RANGE(2)-RADIUS_RANGE(1)) * rand() + RADIUS_RANGE(1);
        % use pt's normal vector and radius to find center.
        ctr = pt - n_pt * rad;
        % DEBUG show the sphere
%         show_sphere(cloud, rad, ctr)

        % find all other pts on sphere. only check nearby pixels.
        r_min = max(1, r - REGION_SIZE); r_max = min(M, r + REGION_SIZE);
        c_min = max(1, c - REGION_SIZE); c_max = min(N, c + REGION_SIZE);
        pts_on_sphere = []; ind_on_sphere = [];
        for i = r_min:r_max
        for j = c_min:c_max
            p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            if all(normals(i,j,:) == 0)
                % pt either has NaNs or isn't part of a surface. skip it.
                continue
            % candidate pt's dist from ctr must be appx 'rad'.
            elseif abs(norm(p_c - ctr) - rad) < EPSILON_RAD
                % normal vector must be appx parallel to vec from ctr.
                n_c = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
                if abs(dot((p_c-ctr)/norm(p_c-ctr), n_c)) > 1 - EPSILON_ANG
                    % candidate could actually be on the sphere.
                    ind_on_sphere = [ind_on_sphere, [i;j]];
                    pts_on_sphere = [pts_on_sphere, p_c];
                end
            end
        end
        end

        % check if enough points fit the sphere.
        if size(pts_on_sphere, 2) > MIN_PTS_ON_SPHERE
            disp(strcat("Found satisfactory sphere on trial ", num2str(trial_num),"."))
            % DEBUG show the sphere.
%             show_sphere(cloud, rad, ctr)
            
            % show just the inliers.
            show_pointcloud(cloud); hold on
            show_inliers(pts_on_sphere)

            % recompute sphere params with only the inliers.
            [center, radius] = snap_sphere(cloud, normals, ind_on_sphere, RADIUS_RANGE);
            % DEBUG show the sphere.
%             show_sphere(cloud, radius, center)
            
            % remove points that are within a larger tolerance as well.
            ind_to_remove = [];
            for i = 1:M
            for j = 1:N
                % check if pt is close to being on sphere.
                p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
                p = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
                if any(isnan(p))
                    continue
                end
                if abs(norm(p_c - center) - radius) < 2*EPSILON_RAD
                    ind_to_remove = [ind_to_remove, [i;j]];
                end
            end
            end
            % ensure these points won't be considered for future planes.
            cloud = strip_inliers(cloud, ind_to_remove);

            % save results and exit RANSAC loop.
            return
        end
    end
    disp(strcat("Failed to find sphere in scene after ",num2str(NUM_RANSAC_TRIALS)," RANSAC iterations."))
end

% Function to refine sphere estimate using set of inliers.
% @param cloud: MxNx3 array of points.
% @param normals: MxNx3 set of surface normal vectors for pts in cloud.
% @param inlier_inds: 2xI set of indices [i;j] of inlier points.
% @param RADIUS_RANGE: 1x2 set of [min,max] allowable radius.
% @return center, radius of sphere.
function [center, radius] = snap_sphere(cloud, normals, inlier_inds, RADIUS_RANGE)
    M = size(inlier_inds, 2);
%     disp(strcat("Number of inliers = ", num2str(M),"."))
    ITERATIONS = 30; iter_num = 0;
    best_error = Inf; best_rad = 0; best_ctr = [0;0;0];
    while iter_num < ITERATIONS
        iter_num = iter_num + 1;
        % choose an inlier to use to calculate sphere params.
        ind = randi([1,M],1);
        r = inlier_inds(1,ind); c = inlier_inds(2,ind);
        pt = [cloud(r,c,1); cloud(r,c,2); cloud(r,c,3)];
        n_pt = [normals(r,c,1); normals(r,c,2); normals(r,c,3)];
        % sample a radius in the range.
        rad = (RADIUS_RANGE(2)-RADIUS_RANGE(1)) * rand() + RADIUS_RANGE(1);
        % use pt's normal vector and radius to find center.
        ctr = pt - n_pt * rad;

        % compute error for all inliers if this is the sphere.
        error = 0;
        for ind = 1:M
            i = inlier_inds(1,ind); j = inlier_inds(2,ind);
            p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            n_c = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
            % pos error is dist to ctr - rad.
            pos_error = abs(norm(p_c - ctr) - rad);
            % angle error is divergence from parallel to sphere surface norm.
            ang_error = 1 - abs(dot((p_c-ctr)/norm(p_c-ctr), n_c));
            % update error for this sphere.
            error = error + pos_error + 0.1 * ang_error;
        end

        % if this is the best sphere so far, save its params.
        if error < best_error
            best_error = error;
            best_rad = rad; best_ctr = ctr;
        end
    end
    % return the best sphere params found.
    radius = best_rad; center = best_ctr;
end

% Function to perform RANSAC to find all planes in a scene.
% @param cloud: MxNx3 array of points.
% @param normals: MxNx3 set of surface normal vectors for pts in cloud.
% @param NUM_PLANES: int. (Optional). number of planes to find in scene, if known.
% @return planes: 1xP cell array of planes. Each cell is 3xN set of pts.
function [planes, cloud] = ransac_planes(cloud, normals, NUM_PLANES)
    if ~exist('NUM_PLANES','var')
        NUM_PLANES = Inf;
    end
    M = size(cloud, 1); N = size(cloud, 2);
    num_planes = 0; planes = {};
    NUM_RANSAC_TRIALS = 1000; trial_num = 1;
    MIN_PTS_IN_PLANE = 19000; 
    EPSILON_POS = 0.015; EPSILON_ANG = 0.1;
    while trial_num < NUM_RANSAC_TRIALS && num_planes < NUM_PLANES
        % choose 3 points at random, assume they form a plane, and find all
        % points that could lie on that plane. 
        pts = [];
        while size(pts, 2) < 3
            % choose a pt at random. ensure it has XYZ.
            r = randi([1,M],1); c = randi([1,N],1);
            if ~any(isnan(cloud(r,c,:)))
                p = [cloud(r,c,1); cloud(r,c,2); cloud(r,c,3)];
                pts = [pts, p];
            end
        end
        % find normal vector for plane.
        v1 = pts(:,2) - pts(:,1);
        v2 = pts(:,3) - pts(:,1);
        n = cross(v1, v2); % using 1st pt as point.
        n = n / norm(n); % make unit vector.
        % eq. of plane: Ax+By+Cz+D=0.
%         A = n(1); B = n(2); C = n(3);
%         D = -A*pts(1,1) - B*pts(2,1) - C*pts(3,1);
        % dist of a point x to the plane is then:
        plane_eq = @(x) dot(n, x) - dot(n, pts(:,1));
        
        % DEBUG show the plane.
%         show_triangle(cloud,pts)

        % for every point, if its unit vector is appx perpendicular to this
        % plane, and its position is close, we say it is included.
        ind_in_plane = []; pts_in_plane = [];
        for i = 1:M
        for j = 1:N
            % check if pt is close to being on plane.
            p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            if any(isnan(p_c))
                continue
            end
            if abs(plane_eq(p_c)) < EPSILON_POS
                % check if pt's surface norm is appx parallel to plane's.
                if all(normals(i,j,:) == 0)
                    % pt is not part of a surface.
                    continue
                end
                nml = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
                if abs(dot(n, nml)) > 1 - EPSILON_ANG
                    % point's surface normal vector is good.
                    ind_in_plane = [ind_in_plane, [i;j]];
                    pts_in_plane = [pts_in_plane, p_c];
                end
            end
        end
        end
        % see if we're satisfied with the number of points in this plane.
        if size(pts_in_plane, 2) > MIN_PTS_IN_PLANE
            disp(strcat("Found satisfactory plane on trial ", num2str(trial_num),"."))
            % DEBUG show the plane.
            show_triangle(cloud, pts)

            % show just the inliers.
            show_pointcloud(cloud); hold on
            show_inliers(pts_in_plane)

            num_planes = num_planes + 1;
            % TODO save the plane (A,B,C,D), not just the set of pts.
            planes{num_planes} = pts_in_plane;

            % remove points that are within a larger tolerance of the plane as well.
            ind_to_remove = [];
            for i = 1:M
            for j = 1:N
                % check if pt is close to being on plane.
                p_c = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
                if any(isnan(p_c))
                    continue
                end
                if abs(plane_eq(p_c)) < 2 * EPSILON_POS
                    ind_to_remove = [ind_to_remove, [i;j]];
                end
            end
            end
            % ensure these points won't be considered for future planes.
            cloud = strip_inliers(cloud, ind_to_remove);
        end
        trial_num = trial_num + 1;
    end
end

% Function to remove points from a cloud by setting them to NaN.
% @param cloud: MxNx3 pointcloud.
% @param indices: 2xI set of [row;column] of pts to remove.
% @return cloud with points removed.
function cloud = strip_inliers(cloud, indices)
    for ind = 1:size(indices, 2)
        cloud(indices(1,ind),indices(2,ind),:) = [nan;nan;nan];
    end
end

% Function to compute surface normal for all points in a pointcloud.
% @param cloud: MxNx3 set of points forming a pointcloud.
% @param USE_PRECOMPUTED_NORMALS: boolean. True will create/use
%        normals.mat. False will always compute them from cloud.
% @return normals: MxNx3 set of surface normal vectors for pts in cloud.
function normals = compute_all_normals(cloud, USE_PRECOMPUTED_NORMALS)
    % compute and save to file to save time on repeated V3 runs.
    try
        if USE_PRECOMPUTED_NORMALS
            load normals.mat normals;
            disp("Found normals.mat. Delete if using a new pointcloud.")
        else
            throw(MException(".","Skipping search for normals.mat."))
        end
    catch
        % either file was not found, or we want to generate fresh.
        % compute the normals for all pts.
        M = size(cloud, 1); N = size(cloud, 2);
        % compute normal vectors for all points.
        normals = zeros(M,N,3);
        for i = 1:M
        for j = 1:N
            normals(i,j,:) = surface_norm(i, j, cloud);
        end
        end
        disp("Finished computing surface norms.")
        if USE_PRECOMPUTED_NORMALS
            % save points to .mat to not have to do this every time.
            save('normals.mat', 'normals')
            disp("Saving normals to normals.mat to speed up future runs.")
        end
    end
end

% Function to compute surface normal at a given point in a pointcloud.
% @param r, c: indexes of point in cloud.
% @param cloud: MxNx3 set of points forming a pointcloud.
% @return normal: 3x1 normal vector of surface at pt.
%  - normal = [0;0;0] if pt is likely not on a surface or contains NaN.
function normal = surface_norm(r, c, cloud)
    % just return if this pt isn't defined.
    if any(isnan(cloud(r,c,:)))
        normal = [0;0;0];
        return
    end
    % extract XYZ of interest pt.
    pt = [cloud(r,c,1); cloud(r,c,2); cloud(r,c,3)];
    % get characteristics of cloud size.
    M = size(cloud, 1); N = size(cloud, 2);
    REGION_SIZE = 2; % radius of square around pt of pixels to check.
    r_min = max(1, r - REGION_SIZE); r_max = min(M, r + REGION_SIZE);
    c_min = max(1, c - REGION_SIZE); c_max = min(N, c + REGION_SIZE);
    % construct a ball around pt, and get all points in cloud w/in ball.
    RADIUS = 0.05; % euclidean distance, radius of ball.
    nbrs = [];
    for i = r_min:r_max
        for j = c_min:c_max
            % if this pixel's XYZ is in the ball, add it as a neighbor.
            p = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            if norm(pt - p) < RADIUS
                nbrs = [nbrs, p];
            end
        end
    end
    % compute mean of points in neghborhood.
    p_mean = [sum(nbrs(1,:)); sum(nbrs(2,:)); sum(nbrs(3,:))] / size(nbrs, 2);
    % calculate sample covariance matrix of pts in neighborhood.
    S = zeros(3,3);
    for i = 1:size(nbrs, 2)
        S = S + (nbrs(:,i) - p_mean) * (nbrs(:,i) - p_mean)';
    end
    [V, D] = eig(S); % gives us eigs in ascending order.
    % if this is actually a surface, there should be two large eigval and 1
    % small. surface nurmal is dir of eigvec for smallest eigval.
    OUTLIER_THRESHOLD = 0.99;
    if D(1,1) / D(3,3) > OUTLIER_THRESHOLD
        % this is likely an outlier, not actually a plane or edge.
        normal = [0;0;0];
    else
        % this is actually a surface. return unit vector.
        normal = V(:,1); % already unit length.
    end
end

%%%%%%%%%%%%%%%%%%% VISUALIZATION FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%

% Function to display a pointcloud in a specified color.
% @param cloud: MxNx3 organized pointcloud to display.
% @param color: string, e.g. 'b','r','g','k','y'.
function show_pointcloud(cloud, color)
    if ~exist('color', 'var')
        color = 'black';
    end
    % display whole cloud in its own figure.
    figure; pcshow(cloud,color,'MarkerSize',12); 
    % make the figure look better visually.
    set(gcf,'color','w'); set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
end

% Function to display a trianglular plane defined by three points.
% @param cloud: MxNx3 organized pointcloud to display.
% @param pts: 3x3 set of three column vectors defining the triangle.
function show_triangle(cloud, pts)
    show_pointcloud(cloud); hold on
    fill3(pts(1,:),pts(2,:),pts(3,:),'red')
end

% Function to display pts categorized into different planes.
% @param planes: 1xP cell array. Each cell is 3xN set of pts.
function show_planes(planes)
    % show each plane in diff color.
    P = size(planes, 2);
    colors = ['r','g','b','c','m','y','k'];
    figure; hold on
    for i = 1:P
        % colors will loop if there are too many planes for unique colors.
        pcshow(planes{mod(i,size(colors,2))}',colors(i),'MarkerSize',12);
    end
    % make the figure white.
    set(gcf,'color','w'); set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
end

% Function to display a pointcloud with a sphere at a certain position.
% @param cloud: MxNx3 organized pointcloud to display.
% @param radius: float. radius of sphere to display.
% @param center: 3x1 point of center of sphere.
function show_sphere(cloud, radius, center)
    show_pointcloud(cloud); hold on
    [X,Y,Z] = sphere(20);
    % scale and move to our desired radius, center.
    X = X * radius + center(1);
    Y = Y * radius + center(2);
    Z = Z * radius + center(3);
%     plot3(X,Y,Z);
    mesh(X,Y,Z)
end

% Function to display a pointcloud with a cylinder at a certain position.
% @param cloud: MxNx3 organized pointcloud to display.
% @param center: 3x1 point of center of cylinder.
% @param radius: float. radius of cylinder.
% @param length: float. distance between circular faces.
% @param axis: 3x1 unit vector describing orientation of axis.
% @param plot_title: string. optional title for plot.
function show_cylinder(cloud, center, radius, length, axis, plot_title)
    if ~exist('plot_title','var')
        plot_title = "Cylinder detected in scene";
    end
    show_pointcloud(cloud); 
    % set the title.
    title(plot_title);
    hold on
    % create cylinder of height 1, with base at origin.
    n = 20;
    [X,Y,Z] = cylinder(radius, n);
    % center it vertically and scale to proper height.
    Z = (Z - 0.5) * length;
    % transform to proper orientation of axis.
    x = axis(1); y = axis(2); z = axis(3);
    R = inv([(x^2*z+y^2)/(x^2+y^2), x*y*(z-1)/(x^2+y^2), -x;
         x*y*(z-1)/(x^2+y^2), (y^2*z+x^2)/(x^2+y^2), -y;
         x, y, z]);
    for i = 1:2
    for j = 1:(n+1)
        pt = [X(i,j); Y(i,j); Z(i,j)];
        pt = R * pt;
        X(i,j) = pt(1); Y(i,j) = pt(2); Z(i,j) = pt(3);
    end
    end
    % move to our desired center pt.
    X = X + center(1);
    Y = Y + center(2);
    Z = Z + center(3);
%     plot3(X,Y,Z);
    mesh(X,Y,Z)
    % plot the axis line.
%     plot3([center(1),center(1)+axis(1)],[center(2),center(2)+axis(2)],[center(3),center(3)+axis(3)])
end

% Function to display pts categorized as inliers (or any set of pts).
% @param inliers: 3xN set of pts.
function show_inliers(inliers)
%     figure; hold on
    pcshow(inliers','r','MarkerSize',12);
    % make the figure white.
    set(gcf,'color','w'); set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
    % set the title.
    title('Inliers');
end

