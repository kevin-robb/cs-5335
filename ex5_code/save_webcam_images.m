% Code to use my laptop's built-in webcam to save pictures.
% webcamlist
% cam = webcam;
% preview(cam);

% counter = 1;

function counter = save_webcam_images(cam, count)
    img = snapshot(cam);
    % save img to file.
    imwrite(img, strcat("book_images/webcam_capture_",num2str(count),".jpg"));  %"Quality", 100
    % increment counter.
    counter = count + 1;
end

% clear cam