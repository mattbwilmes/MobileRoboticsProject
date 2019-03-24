%{
This script processes any dataset of rgb and depth images found here:
https://vision.in.tum.de/data/datasets/rgbd-dataset/download

The user must download the dataset as a .tgz file and unzip it in the
proper folder structure:
-->dataset_path
---->dataset_name
------>unzipped contents of .tgz file
------>assoc.txt (provided by user)

%Author: Matt Wilmes
%Date: 03-24-2019
%}

% Uncomment this line if you need to generate point clouds from the assoc.txt file
%generate_pointclouds;

% Get the directory info of all .pcd files in the pcd_full_path folder
pcd_full_dir_info = dir(fullfile(pcd_full_path, '*.pcd'));
% Make a cell of these filepath names
num_pcd_files = length(pcd_full_dir_info);
ptcloud_files = cell(num_pcd_files,2);
for iter = 1:num_pcd_files    
    % Add .pcd file to ptcloud_files cell
    ptcloud_files{iter,1} = fullfile(pcd_full_path, pcd_full_dir_info(iter).name);
    % Add corresponding rgb file to ptcloud_files cell
    ptcloud_files{iter,2} = fullfile(dataset_path, assoc(iter,2));
end

% Construct an instance of the rgbd_dvo class
rgbd_dvo = rgbd_dvo();

% Load first .pcd file as the target (fixed) point cloud
target_ptcloud = [];
% % Load the point cloud
target_ptcloud.ptcloud = pcread(ptcloud_files{1,1});
% % Load the corresponding rgb image as grayscale
target_ptcloud.image = rgb2gray(imread(ptcloud_files{1,2}));
% Load second .pcd file as the source (moving) point cloud
source_ptcloud = pcread(ptcloud_files{2,1});
%source_ptcloud = [];
%source_ptcloud.ptcloud = pcread(ptcloud_files{2,1});
%source_ptcloud.image = rgb2gray(imread(ptcloud_files{2,2}));

% Add the target and source point clouds to the object
rgbd_dvo.set_ptclouds(target_ptcloud, source_ptcloud);

% Align the point clouds
rgbd_dvo.align();

% Print transform
rgbd_dvo.tform
