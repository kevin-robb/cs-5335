% Code for V1.
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
%%%%%%%%%%%%%%%%%% DETECT FEATURES IN EACH IMAGE %%%%%%%%%%%%%%%%%%
% first get features from template.
sf_template = isurf(I_template, 'nfeat', 1000);
% show template with its features.
figure;
idisp(I_template); sf_template.plot_scale('g');

% get features in each image and compare to template.
for n = 1:num_images
    I = readimage(scene, n);
    % detect SURF features.
    sf = isurf(I, 'nfeat', 1000);
    % show features on image.
    figure;
    idisp(I); sf.plot_scale('g');
    % match features with template.
    [m,correspondences] = sf_template.match(sf);
    % TODO use RANSAC to compute homography between template and I.
    % TODO decompose homography into rotation and translation.
    if n == 1
%         tf_prev = tf;
        correspondences_prev = correspondences;
%         I_prev = I;
%         sf_prev = sf;
        continue
    end
    % use tf_prev and tf to compute I_prev -> I.
    % TODO if tf in SE(3),
%     tf_images = inv(tf_prev) * tf;
    % TODO to display matches, maybe use correspondences for each to match
    % features between the images, using template as intermediary.
end
