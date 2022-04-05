% Code for V1.
%%%%%%%%%%%%%%%%%%%%%%% READ IN IMAGES %%%%%%%%%%%%%%%%%%%%%%%%%%
% im1 = iread('filename1.jpg','mono','double');
% im2 = iread('filename1.jpg','mono','double');
image_dir = "book/";
scene = imageDatastore(strcat('Coursework/cs5335/ex5_code/',image_dir));
% display images in one figure.
montage(scene.Files)
num_images = numel(scene.Files);
% read in template of book cover.
I_template = iread('filename1.jpg','mono','double');
%%%%%%%%%%%%%%%%%% DETECT FEATURES IN EACH IMAGE %%%%%%%%%%%%%%%%%%
% sf_prev = null; I_prev = null;
for n = 1:num_images
    I = readimage(scene, n);
    % detect SURF features.
    sf = isurf(I, 'nfeat', 200);
    % show features on image.
    idisp(I); sf.plot_scale('g');
    if n == 1
        I_prev = I;
        sf_prev = sf;
        continue
    end
    % match pairs of images.
    [m,correspondences] = sf_prev.match(sf);
    % show matches.
    idisp({I_prev, I}); %,'dark'
    m.subset(100).plot('w');
    % TODO use RANSAC to compute homography between I_prev and I.
    % TODO decompose homography into rotation and translation.
end
