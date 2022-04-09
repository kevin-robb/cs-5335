% Code for V3, RANSAC implementation for detecting geometries in scene.
close all; clear all;
%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('ptcloud.mat'); % two MxNx3 pcs called "ptcloud_rgb" and "ptcloud_xyz".
try
    load('points.mat'); % 3xN pointcloud
catch
    % just get the set of points.
    M = size(ptcloud_xyz, 1); N = size(ptcloud_xyz, 2);
    points = [];
    for i = 1:M
        for j = 1:N
            if ~any(isnan(ptcloud_xyz(i,j,:)))
                pt = [ptcloud_xyz(i,j,1); ptcloud_xyz(i,j,2); ptcloud_xyz(i,j,3)];
                points = [points, pt];
            end
        end
    end
    % save points to .mat to not have to do this every time.
    save('points.mat', 'points')
end

% compute planes.
planes = ransac_planes(points);


%%%%%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% display whole cloud.
show_pointcloud(points)

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
% @param cloud: 3xN points to display.
% @param color: string, e.g. 'b','r','g','k','y'.
function show_pointcloud(cloud, color)
    if ~exist('color', 'var')
        color = 'black';
    end
    % display whole cloud in its own figure.
    figure;
    pcshow(cloud',color,'MarkerSize',12); 
    
    % make the figure look better visually.
    set(gcf,'color','w'); set(gca,'color','w');
    set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])
end



% Function to perform RANSAC to find all planes in a scene.
% @param cloud: 3xN array of points.
function planes = ransac_planes(cloud)
    N = size(cloud, 2);
    num_planes = 0; planes = {};
    % compute normal vectors for all points.
    normals = {}; %zeros(3, size(cloud, 2));
    for i = 1:N
        normals{i} = surface_norm(cloud(:, i), cloud);
    end
    NUM_RANSAC_TRIALS = 1000; trial_num = 1;
    MIN_PTS_IN_PLANE = 10000; EPSILON = 0.05;
    while trial_num < NUM_RANSAC_TRIALS && N > MIN_PTS_IN_PLANE
        % choose 3 points at random, assume they form a plane, and find all
        % points that could lie on that plane. 
        indexes = randi([1,N],3,1);
        pts = cloud(:,indexes);
        % find normal vector for plane.
        v1 = pts(:,2) - pts(:,1);
        v2 = pts(:,3) - pts(:,1);
        n = cross(v1, v2); % using 1st vector as point.
        % equation of plane is then: (=0 for points on plane)
        plane_eq = @(x) n(1)*x(1) + n(2)*x(2) + n(3)*x(3) - n(1)*pts(1,1) - n(2)*pts(2,1) - n(3)*pts(3,1);
        % DEBUG show the plane. put breakpoint after this bit.
        show_pointcloud(cloud); hold on
        fill3(pts(1,:),pts(2,:),pts(3,:),rand(1,3))

        % for every point, if its unit vector is appx perpendicular to this
        % plane, and its position is close, we say it is included.
        ind_in_plane = []; pts_in_plane = [];
        for i = 1:N
            % check if pt is close to being on plane.
            if abs(plane_eq(cloud(:,i))) < 0.001
                % check if pt's surface norm is appx parallel to plane's.
                if isempty(normals{i})
                    % pt is not part of a surface.
                    continue
                elseif abs(dot(n, normals{i})) > 1 - EPSILON
                    % point's surface normal vector is good.
                    ind_in_plane = [ind_in_plane, i];
                    pts_in_plane = [pts_in_plane, cloud(:,i)];
                end
            end
        end
        % see if we're satisfied with the number of points in this plane.
        if size(pts_in_plane, 2) > MIN_PTS_IN_PLANE
            num_planes = num_planes + 1;
            % TODO save the plane defn, not just the set of pts.
            planes{num_planes} = pts_in_plane;
            % ensure these points won't be considered for future planes.
            for i = flip(ind_in_plane, 2)
                cloud(:,i) = [];
            end
            % update the size, since it's used as index upper bound.
            N = size(cloud, 2);
        end
        trial_num = trial_num + 1;
    end
end

% Function to compute surface normal at a given point in a pointcloud.
% @param pt: 3x1 point.
% @param cloud: 3xN set of points forming a pointcloud.
% @return normal: 3x1 normal vector of surface at pt.
%  - normal = [] if pt is likely not on a surface.
function normal = surface_norm(pt, cloud)
    % construct a ball around pt, and get all points in cloud w/in ball.
    RADIUS = 0.1;
    nbrs = [];
    for i = 1:size(cloud, 2)
        if norm(pt - cloud(:,i)) < RADIUS
            nbrs = [nbrs, cloud(:,i)];
        end
    end
    % compute mean of points in neghborhood.
    p_mean = [sum(cloud(1,:)); sum(cloud(2,:)); sum(cloud(3,:))] / size(nbrs, 2);
    % calculate sample covariance matrix of pts in neighborhood.
    S = zeros(3,3);
    for i = 1:size(nbrs, 2)
        S = S + (pt - p_mean) * (pt - p_mean)';
    end
    [V, D] = eig(S); % gives us eigs in ascending order.
    % if this is actually a surface, there should be two large eigval and 1
    % small. surface nurmal is dir of eigvec for smallest eigval.
%     eigvals = [D(1,1), D(2,2), D(3,3)];
%     l1 = min(eigvals); l1_vec = V(:,find(eigvals==l1));
%     eigvals(eigvals == l1) = [];
%     l1 = min(eigvals); eigvals(eigvals == l1) = [];
    OUTLIER_THRESHOLD = 0.2;
    if D(1,1) / D(3,3) > OUTLIER_THRESHOLD
        % this is likely an outlier, not actually a plane or edge.
        normal = [];
    else
        % this is actually a surface. return unit vector.
        normal = V(:,1) / norm(V(:,1));
    end
end



