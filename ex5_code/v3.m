
load('bunny.mat'); % 3xN data called "bunny".
pt = bunny(:,50);
normal = surface_norm(pt, bunny);


%%%%%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
pcshow(bunny','r','MarkerSize',12); hold on
% show normal vectors for all points.
normals = zeros(3, size(bunny, 2));
for i = 1:size(bunny, 2)
    normals(:, i) = surface_norm(bunny(:, i), bunny);
end
normals = 0.1 * normals + bunny;
pcshow(normals','g','MarkerSize',12)

% make the figure white.
set(gcf,'color','w'); set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])




%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

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



