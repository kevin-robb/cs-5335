% Code for V3, RANSAC implementation for detecting geometries in scene.
close all; clear all;
%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('ptcloud.mat'); % two MxNx3 pcs called "ptcloud_rgb" and "ptcloud_xyz".
% try
%     load('points.mat'); % 3xN pointcloud
% catch
%     % just get the set of points.
%     M = size(ptcloud_xyz, 1); N = size(ptcloud_xyz, 2);
%     points = [];
%     for i = 1:M
%         for j = 1:N
%             if ~any(isnan(ptcloud_xyz(i,j,:)))
%                 pt = [ptcloud_xyz(i,j,1); ptcloud_xyz(i,j,2); ptcloud_xyz(i,j,3)];
%                 points = [points, pt];
%             end
%         end
%     end
%     % save points to .mat to not have to do this every time.
%     save('points.mat', 'points')
% end

% compute planes.
planes = ransac_planes(ptcloud_xyz);


%%%%%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% display whole cloud.
show_pointcloud(ptcloud_xyz)

% % show normal vectors for all points.
% normals = zeros(3, size(ptcloud_xyz, 2));
% for i = 1:size(ptcloud_xyz, 2)
%     normals(:, i) = surface_norm(ptcloud_xyz(:, i), ptcloud_xyz);
% end
% normals = 0.1 * normals + ptcloud_xyz;
% pcshow(normals','g','MarkerSize',12)

P = size(planes, 2);
colors = ['r','g','b','c','m','y','k'];
% show each plane in diff color.
figure; hold on
for i = 1:P
    pcshow(planes{i}',colors(i),'MarkerSize',12);
end
% pcshow(planes{1}','blue','MarkerSize',12); hold on
% if P > 1
%     pcshow(planes{2}','red','MarkerSize',12);
% end
% if P > 2
%     pcshow(planes{3}','yellow','MarkerSize',12);
% end

% make the figure white.
set(gcf,'color','w'); set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])




%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to display a pointcloud in a specified color.
% @param cloud: MxNx3 organized pointcloud to display.
% @param color: string, e.g. 'b','r','g','k','y'.
function show_pointcloud(cloud, color)
    if ~exist('color', 'var')
        color = 'black';
    end
    % display whole cloud in its own figure.
    figure;
    pcshow(cloud,color,'MarkerSize',12); 
    
    % make the figure look better visually.
    set(gcf,'color','w'); set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
end



% Function to perform RANSAC to find all planes in a scene.
% @param cloud: MxNx3 array of points.
function planes = ransac_planes(cloud)
    M = size(cloud, 1); N = size(cloud, 2);
    num_planes = 0; planes = {};
    % compute normal vectors for all points.
    normals = zeros(M,N,3);
    for i = 1:M
        for j = 1:N
            normals(i,j,:) = surface_norm(i, j, cloud);
        end
    end
    disp("Finished computing surface norms.")
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
        
        % DEBUG show the plane. put breakpoint after this bit.
%         show_pointcloud(cloud); hold on
%         fill3(pts(1,:),pts(2,:),pts(3,:),rand(1,3))

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
            % DEBUG show the plane. put breakpoint after this bit.
            show_pointcloud(cloud); hold on
            fill3(pts(1,:),pts(2,:),pts(3,:),rand(1,3))

            num_planes = num_planes + 1;
            % TODO save the plane (A,B,C,D), not just the set of pts.
            planes{num_planes} = pts_in_plane;
            % ensure these points won't be considered for future planes.
            for ind = 1:size(ind_in_plane,2)
                cloud(ind_in_plane(1,ind),ind_in_plane(2,ind),:) = [nan;nan;nan];
            end
%             % update the size, since it's used as index upper bound.
%             N = size(cloud, 2);
        end
        trial_num = trial_num + 1
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


