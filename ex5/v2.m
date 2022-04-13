% Code to perform ICP for V3.
close all; clear all;
%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% read in pointcloud data.
load('bunny.mat'); % 3xN data called "bunny".
N = size(bunny, 2);
% create new pointcloud by transforming this one.
tf_true = SE3.rand().T;
bunny2 = transform_points(tf_true,bunny);
% add perturbations to the individual points.
bunny2 = bunny2 + 0.01 * randn(3, N);
% add spurious points and remove some existing points.
% we will add points near the existing points.
num_points_to_add = round(N / 3);
cg_2 = [sum(bunny2(1,:)); sum(bunny2(2,:)); sum(bunny2(3,:))] / N;
new_points = cg_2 + 0.1 * randn(3,num_points_to_add);
bunny2 = [bunny2, new_points];
% remove the same # of points at random.
indexes = randi([1,N+num_points_to_add],num_points_to_add,1);
for i = indexes
    bunny2(:,i) = [];
end

% make initial guess for transform. if rotation is too different,
% there is no hope of ICP working.
tf_guess = tf_true + 0.2 * SE3.rand().T;
% use ICP to estimate the transform.
tf_est = icp(bunny, bunny2, tf_guess);

%%%%%%%%%%%%%%%%%%%%%%% SHOW RESULTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
pcshow(bunny','r','MarkerSize',12); hold on
% compute transformed bunny with our tf estimate.
bunny2_est = transform_points(tf_est, bunny);
pcshow(bunny2_est','g','MarkerSize',12)
% show true transformed bunny.
pcshow(bunny2','b','MarkerSize',12)

% make the figure white.
set(gcf,'color','w'); set(gca,'color','w');
set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15])


% print transforms to console.
tf_true
tf_est


%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to apply a transform to a set of points.
% @param tf: 4x4 transform matrix.
% @param pts: 3xN array of points to transform.
% @return tf_pts: 3xN array of transformed pts.
function tf_pts = transform_points(tf, pts)
    tf_pts = zeros(3,size(pts,2));
    for i=1:size(pts,2)
        % perform affine transformation.
        affine_pt = tf * [pts(:,i); 1];
        % remove row of 1s.
        tf_pts(:,i) = affine_pt(1:3);
    end
end

% Function to estimate the transform between two pointclouds
% using the iterative closest point (ICP) algorithm.
% @param set_1, set_2: two 3xN arrays for the two pointclouds.
% @param tf_init: initial guess for 4x4 transformation matrix.
%  - default = eye(4)
% @return transform: 4x4 homogenous transformation matrix.
function transform = icp(set_1, set_2, transform)
    % keep track of point sets as affine representation to allow tf mult.
    N = size(set_1, 2);
    prev_set = [set_1; ones(1, N)];
    set_2 = [set_2; ones(1, size(set_2, 2))];
    % need initial estimate for transform.
    if ~exist('transform', 'var')
        transform = eye(4); % identity in SE(3).
    end
    % parameters
    diff = 100; TOLERANCE = 0.00001;
    iter_count = 1; MAX_ITERATIONS = 100;
    % iteratively correlate points in set_1 with nearest neighbor in set_2,
    % estimate a transformation, and compute the error.
    while diff > TOLERANCE && iter_count < MAX_ITERATIONS
        % first create data associations with NN method.
        tf_set_1 = ones(4,N);
        neighbors = ones(4,N);
        for i = 1:N
            % compute transform of a point in set_1.
            tf_set_1(:,i) = transform * [set_1(:,i); 1];
            % find nearest neighbor of this point in set_2.
            neighbors(:,i) = nearest_pt(tf_set_1(:,i), set_2);
        end
        % DEBUG show the pointclouds.
%         figure; pcshow(tf_set_1(1:3,:)','g'); hold on
%         pcshow(neighbors(1:3,:)','b')
        % using these data associations, estimate a transform between them.
        tf = estimate_transform(tf_set_1(1:3,:), neighbors(1:3,:));
        % apply this to our cumulative transform.
        transform = tf * transform;
        if iter_count > 1
            % compute the difference between tf_set_1 and prev_set.
            diff = norm(tf_set_1 - prev_set) / N;
        end
        % update the previous set.
        prev_set = tf_set_1;
        % increment the iteration count.
        iter_count = iter_count + 1;
    end
end

% Function to compute the nearest point in set_2 to point p1.
function pt = nearest_pt(p1, set_2)
    min_dist = Inf;
    for i = 1:size(set_2, 2)
        if norm(p1 - set_2(:,i)) < min_dist
            min_dist = norm(p1 - set_2(:,i));
            pt = set_2(:,i);
        end
    end
end

% Function to estimate the homography between two sets of points
% with known data association.
% @param set_1, set_2: two 2xN double arrays.
% @return transform: 4x4 transform matrix which can be 
% applied to set_1 to approximate set_2.
function transform = estimate_transform(set_1, set_2)
    GROUP_SIZE = size(set_1, 2);
    % first, compute center of gravity for each set to get translation.
    cg_1 = [sum(set_1(1,:)); sum(set_1(2,:)); sum(set_1(3,:))] / GROUP_SIZE;
    cg_2 = [sum(set_2(1,:)); sum(set_2(2,:)); sum(set_2(3,:))] / GROUP_SIZE;
    % next, center the sets of points at 0 so we can compute the rotation.
    set_1 = set_1 - cg_1; set_2 = set_2 - cg_2;
    % compute the matrix N.
    W = zeros(3,3);
    for i=1:GROUP_SIZE
        W = W + set_1(:,i) * set_2(:,i)';
    end
    % find the singular value decomposition of N.
    [U,S,V] = svd(W); % N=U*S*V'
    % compute the rotation matrix.
    R = V * U';
    % extract the rotation angle.
%         alpha = atan2(R(2,1), R(1,1));
    % now we can build our SE3 transformation matrix.
%     t1 = SE3(-cg_1(1), -cg_1(2), -cg_1(3));
%     t2 = SE3(R);
%     t3 = SE3(cg_2(1), cg_2(2), cg_2(3));
%     transform = t3 * t2 * t1;
    t1 = [eye(3), -cg_1; zeros(1,3), 1];
    t2 = [R, zeros(3,1); zeros(1,3), 1];
    t3 = [eye(3), cg_2; zeros(1,3), 1];
    transform = t3 * t2 * t1;
end






