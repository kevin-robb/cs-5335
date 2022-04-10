% Code for V3, RANSAC implementation for detecting geometries in scene.
close all; clear all;
%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('ptcloud.mat'); % two MxNx3 pcs called "ptcloud_rgb" and "ptcloud_xyz".
% display whole pointcloud.
% show_pointcloud(ptcloud_xyz)

% compute surface norms.
normals = compute_all_normals(ptcloud_xyz);

%%%%%%%%%%%%%%%%%%%%%%%%% PLANES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute planes.
% planes = ransac_planes(ptcloud_xyz, normals);
% show_planes(planes);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% SPHERE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% find the sphere's center and radius with RANSAC.
[center, radius] = ransac_sphere(ptcloud_xyz, normals);



%%%%%%%%%%%%%%%%%% COMPUTATIONAL FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to perform RANSAC to find the sphere in a scene.
% @param cloud: MxNx3 array of points.
% @param normals: MxNx3 set of surface normal vectors for pts in cloud.
% @return center, radius of sphere.
function [center, radius] = ransac_sphere(cloud, normals)
    M = size(cloud, 1); N = size(cloud, 2);
    RADIUS_RANGE = [0.05, 0.10]; % meters.
    NUM_RANSAC_TRIALS = 500; trial_num = 0;
    MIN_PTS_ON_SPHERE = 6000; 
    EPSILON_RAD = 0.02; EPSILON_ANG = 0.2;
    REGION_SIZE = 50; % radius of area in pixels to search around candidate pt.
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
            % DEBUG show the sphere and iteration count.
%             show_sphere(cloud, rad, ctr)

            % recompute sphere params with only the inliers.
            [radius, center] = snap_sphere(cloud, normals, ind_on_sphere, RADIUS_RANGE);
            % DEBUG show the sphere.
            show_sphere(cloud, radius, center)

            % save results and exit RANSAC loop.
            return
        end
    end
end

% Function to refine sphere estimate using set of inliers.
function [radius, center] = snap_sphere(cloud, normals, inlier_inds, RADIUS_RANGE)
    M = size(inlier_inds, 2);
    disp(strcat("Number of inliers = ", num2str(M),"."))
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
% @return planes: 1xP cell array of planes. Each cell is 3xN set of pts.
function planes = ransac_planes(cloud, normals)
    M = size(cloud, 1); N = size(cloud, 2);
    num_planes = 0; planes = {};
    NUM_RANSAC_TRIALS = 300; trial_num = 1;
    MIN_PTS_IN_PLANE = 20000; 
    EPSILON_POS = 0.01; EPSILON_ANG = 0.1;
    while trial_num < NUM_RANSAC_TRIALS
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
        A = n(1); B = n(2); C = n(3);
        D = -A*pts(1,1) - B*pts(2,1) - C*pts(3,1);
        % dist of a point x to the plane is then:
        plane_eq = @(x) (A*x(1) + B*x(2) + C*x(3) +D);
        
        % DEBUG show the plane.
%         show_triangle(cloud,pts)

        % for every point, if its unit vector is appx perpendicular to this
        % plane, and its position is close, we say it is included.
        ind_in_plane = []; pts_in_plane = [];
        for i = 1:M
        for j = 1:N
            % check if pt is close to being on plane.
            p = [cloud(i,j,1); cloud(i,j,2); cloud(i,j,3)];
            if any(isnan(p))
                continue
            end
            if abs(plane_eq(p)) < EPSILON_POS
                % check if pt's surface norm is appx parallel to plane's.
                if all(normals(i,j,:) == 0)
                    % pt is not part of a surface.
                    continue
                end
                nml = [normals(i,j,1); normals(i,j,2); normals(i,j,3)];
                if abs(dot(n, nml)) > 1 - EPSILON_ANG
                    % point's surface normal vector is good.
                    ind_in_plane = [ind_in_plane, [i;j]];
                    pts_in_plane = [pts_in_plane, p];
                end
            end
        end
        end
        % see if we're satisfied with the number of points in this plane.
        if size(pts_in_plane, 2) > MIN_PTS_IN_PLANE
            disp("Found satisfactory plane.")
            % DEBUG show the plane.
            show_triangle(cloud,pts)

            num_planes = num_planes + 1;
            % TODO save the plane (A,B,C,D), not just the set of pts.
            planes{num_planes} = pts_in_plane;
            % ensure these points won't be considered for future planes.
            for ind = 1:size(ind_in_plane,2)
                cloud(ind_in_plane(1,ind),ind_in_plane(2,ind),:) = [nan;nan;nan];
            end
        end
        trial_num = trial_num + 1;
    end
end

% Function to compute surface normal for all points in a pointcloud.
% @param cloud: MxNx3 set of points forming a pointcloud.
% @return normals: MxNx3 set of surface normal vectors for pts in cloud.
function normals = compute_all_normals(cloud)
    % compute and save to file to save time on repeated V3 runs.
    try
        load normals.mat normals;
        disp("Found normals saved. Delete normals.mat if using a different pointcloud.")
    catch
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
        % save points to .mat to not have to do this every time.
        save('normals.mat', 'normals')
        disp("Saving normals to normals.mat to speed up future runs.")
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
    fill3(pts(1,:),pts(2,:),pts(3,:),rand(1,3))
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




