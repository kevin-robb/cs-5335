% Code for V1.
close all;
%%%%%%%%%%%%%%%%%%%%%%% READ IN IMAGES %%%%%%%%%%%%%%%%%%%%%%%%%%
% im1 = iread('filename1.jpg','mono','double');
% im2 = iread('filename1.jpg','mono','double');
image_dir = "book_images/";
scene = imageDatastore(image_dir);
% scene = imageDatastore(strcat('Coursework/cs5335/ex5_code/',image_dir));
% display images in one figure.
% montage(scene.Files);
num_images = numel(scene.Files);
% read in template of book cover.
I_template = iread('catch_22_template.jpg'); %'mono','double'
I_template = scale_down_img(I_template, 3.7);
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
    figure;
    idisp(I); sf.plot_scale('g');
    % match features with template.
    [m,correspondences] = sf_template.match(sf);
    % show matches.
    idisp({I_template, I}, 'dark');
    m.subset(100).plot('w');
    % TODO use RANSAC to compute homography between template and I.
%     m.ransac(@homography, 1e-4)
    tf_new = tf_ransac(m)
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
% @param matches: set of matching points from template to image.
function transform = tf_ransac(matches)
    % assume a linear affine transform will suffice;
    % i.e., one translation and a rotation about a single point.
    transform = SE2();
    group_size = 10;
    full_set_1 = matches.p1; full_set_2 = matches.p2;
    % TODO loop the rest until tf is good.

    % choose random set of points.
    indexes = randi([1,group_size],group_size,1);
    set_1 = zeros(2,group_size); set_1 = zeros(2,group_size);
    for i = 1:group_size
        set_1(:,i) = full_set_1(:,indexes(i,1));
        set_2(:,i) = full_set_2(:,indexes(i,1));
    end
    % first, compute center of gravity for each set to get translation.
    cg_1 = [sum(set_1(1)); sum(set_1(2))] / group_size;
    cg_2 = [sum(set_2(1)); sum(set_2(2))] / group_size;
    translation = cg_2 - cg_1;
    % next, center the sets of points at 0 so we can compute the rotation.
    set_1 = set_1 - cg_1; set_2 = set_2 - cg_2;
    % compute the matrix N.
    N = zeros(2,2);
    for i=1:group_size
        N = N + set_1(:,i) * set_2(:,i)';
    end
    % find the singular value decomposition of N.
    [U,S,V] = svd(N); % N=U*S*V'
    % compute the rotation matrix.
    R = V * U';
%     % extract the rotation angle.
%     alpha = atan2(R(2,1), R(1,1));
    % now we can build our SE3 transformation matrix.
    t1 = SE2(-cg_1(1), -cg_1(2));
    t2 = SE2(R);
    t3 = SE2(cg_2(1), cg_2(2));
    transform = t1 * t2 * t3;
end


% Function to scale down an image by N.
function outputImage = scale_down_img(inputImage, N)
    [rows, columns, ~] = size(inputImage);
    numOutputRows = round(rows/N);
    numOutputColumns = round(columns/N);
    outputImage = imresize(inputImage, [numOutputRows, numOutputColumns]);
end
