% Code for V1.
close all;
%%%%%%%%%%%%%%%%%%%%%%% READ IN IMAGES %%%%%%%%%%%%%%%%%%%%%%%%%%
% im1 = iread('filename1.jpg','mono','double');
% im2 = iread('filename1.jpg','mono','double');
image_dir = "book_images/";
scene = imageDatastore(image_dir);

%-----------------------------------------------------------------
% % Even his example in the ransac function does not execute without errors.
% f1 = isurf(readimage(scene,1));
% f2 = isurf(readimage(scene,2));
% m = f1.match(f2);
% m.ransac( @fmatrix, 1e-4);
%-----------------------------------------------------------------

% scene = imageDatastore(strcat('Coursework/cs5335/ex5_code/',image_dir));
% display images in one figure.
% montage(scene.Files);
num_images = numel(scene.Files);
% read in template of book cover.
I_template = iread('catch_22_template.jpg'); %'mono','double'
I_template = scale_down_img(I_template, 3.7);
t_h = size(I_template, 1); t_w = size(I_template, 2);
%%%%%%%%%%%%%%%%%% DETECT FEATURES IN EACH IMAGE %%%%%%%%%%%%%%%%%%
% first get features from template.
sf_template = isurf(I_template, 'nfeat', 800);
% show template with its features.
figure;
idisp(I_template); sf_template.plot_scale('g');

% get features in each image and compare to template.
for n = 1:num_images
    I = readimage(scene, n);
    % detect SURF features.
    sf = isurf(I, 'nfeat', 800);
    % show features on image.
%     figure;
%     idisp(I); sf.plot_scale('g');
    % match features with template.
    [m,correspondences] = sf_template.match(sf);
    % show matches.
    figure;
    idisp({I_template, I}, 'dark');
    m.subset(100).plot('w');
    % TODO use RANSAC to compute homography between template and I.
%     m.ransac(@fmatrix, 1e-4)
    %--------------------------------------------------------------
    [transform, t1, t2, t3] = tf_ransac(m);
    % show these transformed points on the plot for visual inspection.
    hold on
    tf_pts = m.p1;
    for tf = [t1, t2, t3]
        tf_pts = transform_points(tf,tf_pts);
        scatter(tf_pts(1,:)+t_w, tf_pts(2,:))
    end
    % show the transformed points on just the real image.
    figure; idisp(I); hold on
    scatter(tf_pts(1,:), tf_pts(2,:))
    % show an outline of the book on the real image by transforming the
    % outline of the template.
    p = 20; % padding from edges of template.
    corners = [p, p; p, t_h-p; t_w-p, t_h-p; t_w-p, p; p, p]';
    tf_corners = transform_points(transform, corners);
%     for i = 1:4
%         plot()
%     end
    plot(tf_corners(1,:), tf_corners(2,:))
    %--------------------------------------------------------------
    % TODO decompose homography into rotation and translation.
    if n > 1
        % use tf_prev and tf to compute I_prev -> I.
        % TODO if tf in SE(3),
    %     tf_images = inv(tf_prev) * tf;
        % TODO to display matches, maybe use correspondences for each to match
        % features between the images, using template as intermediary.
    end
    % save things for "previous image" on next iteration.
%     tf_prev = tf;
    correspondences_prev = correspondences;
    I_prev = I;
    sf_prev = sf;
end



%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%
% Function to perform RANSAC to obtain a transform between pairs of images.
% @param matches: FeatureMatch object from template to image.
function [transform, t1, t2, t3] = tf_ransac(matches)
    % config parameters.
    GROUP_SIZE = 40;
    EPSILON = 40; % (in pixels) tolerance for a tf being ok for a pair.
    TAU = 0.5; % threshhold for ratio that tf works for. (ideally like 0.9)
    % assume a linear affine transform will suffice;
    % i.e., one translation and a rotation about a single point.
    transform = SE2();
    full_set_1 = matches.p1; full_set_2 = matches.p2;
    total_size = size(full_set_1, 2);
    % loop the rest until tf is good.
    ratio_in_tolerance = 0;
    while ratio_in_tolerance < TAU
        % choose random set of points.
        indexes = randi([1,total_size],GROUP_SIZE,1);
        set_1 = zeros(2,GROUP_SIZE); set_2 = zeros(2,GROUP_SIZE);
        for i = 1:GROUP_SIZE
            set_1(:,i) = full_set_1(:,indexes(i,1));
            set_2(:,i) = full_set_2(:,indexes(i,1));
        end
        % first, compute center of gravity for each set to get translation.
        cg_1 = [sum(set_1(1,:)); sum(set_1(2,:))] / GROUP_SIZE;
        cg_2 = [sum(set_2(1,:)); sum(set_2(2,:))] / GROUP_SIZE;
    %     translation = cg_2 - cg_1;
        % next, center the sets of points at 0 so we can compute the rotation.
        set_1 = set_1 - cg_1; set_2 = set_2 - cg_2;
        % compute the matrix N.
        N = zeros(2,2);
        for i=1:GROUP_SIZE
            N = N + set_1(:,i) * set_2(:,i)';
        end
        % find the singular value decomposition of N.
        [U,S,V] = svd(N); % N=U*S*V'
        % compute the rotation matrix.
        R = V * U';
    %     % extract the rotation angle.
%         alpha = atan2(R(2,1), R(1,1));
        % now we can build our SE3 transformation matrix.
        t1 = SE2(-cg_1(1), -cg_1(2));
        t2 = SE2(R);
        t3 = SE2(cg_2(1), cg_2(2));
        transform = t3 * t2 * t1;
        
        % check how many points fit within a tolerance using this tf.
        num_in_tolerance = 0;
%         transformed_set_1 = zeros(2,total_size);
        for i = 1:total_size
            affine_tf_pt = transform.T * [full_set_1(:,i); 1];
%             transformed_set_1(:,i) = affine_tf_pt(1:2);
            if EPSILON > norm(affine_tf_pt(1:2) - full_set_2(:,i))
                num_in_tolerance = num_in_tolerance + 1;
            end
        end
        % compute ratio of inliers to total points.
        ratio_in_tolerance = num_in_tolerance / total_size;
    end
%     % show matches from transform.
%     tf_matches = FeatureMatch(matches.p1, transformed_set_1);
% %     matches.p2 = transformed_set_1;
%     figure;
%     idisp({I_template, I}, 'dark');
%     tf_matches.subset(100).plot('w');
end


% Function to apply a transform to a set of points.
% @param tf: SE2 transform.
function tf_pts = transform_points(tf, pts)
    tf_pts = zeros(2,size(pts,2));
    for i=1:size(pts,2)
        % perform affine transformation.
        affine_pt = tf.T * [pts(:,i); 1];
        % remove row of 1s.
        tf_pts(:,i) = affine_pt(1:2);
    end
end


% Function to scale down an image by N.
function outputImage = scale_down_img(inputImage, N)
    [rows, columns, ~] = size(inputImage);
    numOutputRows = round(rows/N);
    numOutputColumns = round(columns/N);
    outputImage = imresize(inputImage, [numOutputRows, numOutputColumns]);
end
