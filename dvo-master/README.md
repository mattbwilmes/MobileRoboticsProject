# DVO Method

Based on paper by Steinbrücker et al. called "Real-time Visual Odometry from Dense RGB-D Images"
- https://ieeexplore.ieee.org/abstract/document/6130321

Tested with: MATLAB 2018b

## Required uncommon MATLAB package:
- Computer Vision Toolbox

## Procedure:
1. Open rgbd_main.m
2. Change rgbd_tum_path to match the location of the dvo-master/rgbd_tum folder
3. Press Run
4. Observe the improvement with versus without the transform between the first four images
5. Frame-to-frame transformations are saved as dvo-master/rgbd_tum/[datasest]/[dataset]_tform_new.mat

## Folder structure
dvo-master

-->@rgbd_dvo/

---->rgbd_dvo.m - rgbd_dvo clas

-->rgbd_tum/

---->freiburg1_desk/ - see freiburg1_xyz for structure

---->freiburg1_xyz/

------>scale_1/ - Full scale images and point clouds

-------->depth/ - depth images

-------->pcd_edge/ - edge point clouds

-------->pcd_full/ - full point clouds

-------->rgb/ - rgb images

-------->assoc.txt - file used to match depth and rgb images

-------->depth.txt - input to generate assoc.txt

-------->rgb.txt - input to generate assoc.txt

------>scale_05/ - Pre-generated 0.5 scale images and point clouds

------>scale_025/ - Pre-generated 0.5 scale images and point clouds

---->rgbd_benchmark_tools - Contains TUM dataset 

---->pcRangeFilter.m - Filters out distant points

---->ptcloud_edge_filter.m - Generates edge point clouds using Canny method

---->scale_pointclouds_and_images.m - Used to generate scale_05/, ..., scale_0625/ folders

**-->rgbd_main.m - Main script to run**
