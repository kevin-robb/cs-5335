# Instructions to run my code for all of CS-5335 EX5

These steps assume the folder containing my code and all subdirectories have been added to the MATLAB path.

## OPT
### a-b
In the script `opt.m`, choose a cost function by uncommenting one of the lines in the "Choose Cost Function" section, clearly delineated on lines 16-20. The first line is for part (a), and the next three lines are other cost functions for part (b).

After selecting a cost function in this way, simply run the script by typing `opt.m` into the MATLAB command window.

### c-e
Run this script with `opt_lightdark.m` in the command window. Changes made for part (e) were not left in, other than two options for the initial trajectory estimate on lines 5-6.

## V1
I've included a folder with some images I took with my webcam, as well as the template I used for my book cover. As such, this code can be run with `v1.m`. The translation and rotation between each pair of images will be printed to the console as requested. Many figures will appear as well.

For part (e), uncomment line 12 to load the webcam. This requires MATLAB's support package for USB webcams. I've tried to make it as elegant as possible, so nothing else needs to be changed and it should work fine.

## V2
Run this script with `v2.m`. A pointcloud figure will appear showing the initial, true transformed, and estimated transformed cloud. The true and estimated transforms will both print to the console.

Perturbations and noise introduced for part (c) was left in, in the noisiest state that still pretty much works. To see it perform without the noise and spurious and missing points, comment out lines 10-22.

## V3
Run `v3.m` to run all of V3 in series. This will compute all surface normals, find the planes, find the sphere, and then find the cylinder. In all, it will probably take 1 or 2 minutes to run. After each geometry detection, its inliers are removed from the pointcloud to prevent repeated classification.

The detectors will print to the console the trial at which they found success, and many plots will appear showing the inliers and geometries for each. The final parameters for the geometries are not displayed, but are saved as variables when the script finishes, so you can see them all with `who` and see the value for a particular variable by typing its name in the command window.

To see each geometry on its own, comment out the other two sections in the line 13-29 region before running.
