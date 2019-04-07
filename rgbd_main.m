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

clc; clear; close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User Input Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RGB intrinsic calibration parameters
% Freiburg 1
fx = 517.3;  % focal length x
fy = 516.5;  % focal length y
cx = 318.6;  % optical center x
cy = 255.3;  % optical center y

% Freiburg 2
% fx = 520.9;  % focal length x
% fy = 521.0;  % focal length y
% cx = 325.1;  % optical center x
% cy = 249.7;  % optical center y

% Freiburg 3
% fx = 535.4;  % focal length x
% fy = 539.2;  % focal length y
% cx = 320.1;  % optical center x
% cy = 247.6;  % optical center y

scaling_factor = 5000;  % depth scaling factor

% % TODO: Make this a custom path based on user
% Path to the folder container your rgbd_tum datasets
rgbd_tum_path = ...
    '/Users/MatthewWilmes/Documents/MATLAB/School/EECS568_MATLAB/Project/rgbd_tum/';
% Select the dataset you want to use
dataset_name = 'freiburg1_xyz';
% The extension of the images in the dataset (e.g. png, jpeg, etc.)
image_suffix = '.png';

% Set this flag to true if you have not made .pcd files for your dataset yet
make_pcd_files = false;
% If make_pcd_files is true, set the first row and last row you want to
%   read from the assoc.txt file
start_row = 1; % set to 1 if you want to start at the beginning of the file
end_row = 2; % set to inf if you want to go to the end of the file

% Set this flag to true if you want to see the full point cloud output
%   instead of the edges. NOTE: This is only useful for frames that have a
%   large offset
view_full_ptcloud = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate name of folder path to dataset
dataset_path = strcat(rgbd_tum_path, dataset_name, '/');
% Generate names of folders to .pcd files
pcd_full_path = strcat(dataset_path,'pcd_full');
pcd_edge_path = strcat(dataset_path,'pcd_edge');

% Generate .pcd files from dataset if true
if make_pcd_files
    generate_pointclouds(fx, fy, cx, cy, scaling_factor, ...
        dataset_path, pcd_full_path, pcd_edge_path, start_row, end_row);
end

% Get the directory info of all .pcd files in the pcd_edge_path folder
pcd_edge_dir_info = dir(fullfile(pcd_edge_path, '*.pcd'));
% Make a cell of these filepath names
num_pcd_files = length(pcd_edge_dir_info);
ptcloud_files = cell(num_pcd_files,2);
% Remove non-alphabet letters from image extension in case a period was
%   included
image_suffix(isletter(image_suffix)==0)=[];

for iter = 1:num_pcd_files
    % Get name of the iter'th pcd file
    pcd_file = pcd_edge_dir_info(iter).name;
    % Get name of the corresping rgb file
    rgb_file = replace(pcd_file,'pcd',image_suffix);
    % Add .pcd file to ptcloud_files cell
    ptcloud_files{iter,1} = fullfile(pcd_edge_path, pcd_file);
    % Add corresponding rgb file to same row of ptcloud_files cell
    ptcloud_files{iter,2} = fullfile(dataset_path, 'rgb/', rgb_file);
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

% Make a temporary target point cloud for down-sampling
target_ptcloud_temp.image = target_ptcloud.image;
target_ptcloud_temp.ptcloud = target_ptcloud.ptcloud;
% Make a temporary source point cloud for down-sampling
source_ptcloud_temp = source_ptcloud;

tic
% Coarse-to-fine approach
for grid_step = 0.1:-0.001:0.01
%for grid_step = 0.1:-0.01:0.01
%for grid_step = 0.1:-0.025:0.05
%for grid_step = [0.1 0.0 0.09 0.08 0.07 0.06 0.05 0.025 0.01 0.005 0.001]
%grid_step = 0.1
%while grid_step > 0.01
    % Adjust tolerances based on value of grid_step
    if mod(round(grid_step,4),0.01) == 0
        rgbd_dvo.eps = 5*1e-5;
        rgbd_dvo.eps_2 = 1e-5;
        grid_step
    else
        rgbd_dvo.eps = 5*1e-4;
        rgbd_dvo.eps_2 = 1e-4;
        grid_step
    end
    % Down-sample target point cloud
    target_ptcloud_temp.ptcloud = pcdownsample(target_ptcloud_temp.ptcloud,'gridAverage',grid_step);
    % Down-sample source point cloud
    source_ptcloud_temp = pcdownsample(source_ptcloud_temp,'gridAverage',grid_step);

%     rgbd_dvo.set_ptclouds(target_ptcloud, source_ptcloud);
    % Add the target and source point clouds to the object
    rgbd_dvo.set_ptclouds(target_ptcloud_temp, source_ptcloud_temp);
    
    % Align the point clouds
    rgbd_dvo.align();
    grid_step
    %grid_step = grid_step / 2
    
end
% rgbd_dvo.set_ptclouds(target_ptcloud,source_ptcloud)
% rgbd_dvo.align();
toc

% Plot full point clouds and transformed point clouds if true
if view_full_ptcloud
    % Generate filenames for the full point clouds
    target_ptcloud_filename = ...
        replace(ptcloud_files{1,1},'pcd_edge','pcd_full');
    source_ptcloud_filename = ...
        replace(ptcloud_files{2,1},'pcd_edge','pcd_full');

    % Read full target and source point clouds
    target_ptcloud_full = pcread(target_ptcloud_filename);
    source_ptcloud_full = pcread(source_ptcloud_filename);
    % Apply final transform to full source point cloud
    source_ptcloud_full_transformed = pctransform(source_ptcloud_full, rgbd_dvo.tform);

    % Plot original point clouds on top of one another to show misalignment
    figure(1)
    pcshow(target_ptcloud_full)
    hold on
    pcshow(source_ptcloud_full)
    view(0,-90)
    title('Target and Source Point Clouds without Transform')

    % Plot point clouds on top of one another to show improved alignment
    figure(2)
    pcshow(target_ptcloud_full)
    hold on
    pcshow(source_ptcloud_full_transformed)
    view(0,-90)
    title('Target and Source Point Clouds with Transform')

% Otherwise, plot edge point clouds and transformed point clouds
else
    % Read full target and source point clouds
    target_ptcloud_edge = pcread(ptcloud_files{1,1});
    source_ptcloud_edge = pcread(ptcloud_files{2,1});
    % Apply final transform to edge source point cloud
    source_ptcloud_edge_transformed = pctransform(source_ptcloud_edge, rgbd_dvo.tform);

    % Plot original point clouds on top of one another to show misalignment
    figure(1)
    pcshow(target_ptcloud_edge)
    hold on
    pcshow(source_ptcloud_edge)
    view(0,-90)
    title('Target and Source Point Clouds without Transform')

    % Plot point clouds on top of one another to show improved alignment
    figure(2)
    pcshow(target_ptcloud_edge)
    hold on
    pcshow(source_ptcloud_edge_transformed)
    view(0,-90)
    title('Target and Source Point Clouds with Transform')
end

disp('done')

% Print transform
%rgbd_dvo.tform.T
