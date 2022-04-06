% Code to perform ICP for V3.
close all;

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


function transform = icp(set_1, set_2)
    % keep track of point sets as affine representation to allow tf mult.
    N = size(set_1, 2);
    prev_set = [set_1; ones(1, N)];
    set_2 = [set_2; ones(1, size(set_2, 2))];
    % TODO need initial estimate for transform.
    transform = SE3();
    TOLERANCE = 0.1;
    diff = 100;
    while diff > TOLERANCE
        % iteratively correlate points in set_1 with nearest neighbor in set_2,
        % estimate a transformation, and compute the error.
        tf_set_1 = ones(4,N);
        neighbors = ones(4,N);
        for i = 1:N
            % compute transform of a point in set_1.
            tf_set_1(:,i) = transform.T * set_1(:,i);
            % find nearest neighbor of this point in set_2.
            neighbors(:,i) = nearest_pt(tf_set_1(:,i), set_2);
        end
        % using these data associations, estimate a transform between them.
        tf = estimate_transform(tf_set_1, neighbors);
        % apply this to our cumulative transform.
        transform = SE3(tf.T * transform.T);
        % compute the difference between tf_set_1 and prev_set.
        diff = norm(tf_set_1 - prev_set) / N;
        % update the previous set.
        prev_set = tf_set_1;
    end
end


% Function to estimate the homography between two sets of points
% with known data association.
% @param set_1, set_2: two 2xN double arrays.
% @return transform: SE2 homography which can be 
% applied to set_1 to approximate set_2.
function transform = estimate_transform(set_1, set_2)
    GROUP_SIZE = size(set_1, 2);
    % first, compute center of gravity for each set to get translation.
    cg_1 = [sum(set_1(1,:)); sum(set_1(2,:))] / GROUP_SIZE;
    cg_2 = [sum(set_2(1,:)); sum(set_2(2,:))] / GROUP_SIZE;
    % next, center the sets of points at 0 so we can compute the rotation.
    set_1 = set_1 - cg_1; set_2 = set_2 - cg_2;
    % compute the matrix N.
    W = zeros(2,2);
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
    t1 = SE2(-cg_1(1), -cg_1(2));
    t2 = SE2(R);
    t3 = SE2(cg_2(1), cg_2(2));
    transform = t3 * t2 * t1;
end