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
%Date: 04-15-2019
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

% Number of times to down-scale the images
% The resulting scale_vec will start with 0.5 and divide by factor of 2 each time
% e.g. scale_num = 4 --> [0.5 0.25 0.125 0.0625]
scale_num = 4;

depth_scaling_factor = 5000;  % depth scaling factor

% % TODO: Make this a custom path based on user
% Path to the folder containing your rgbd_tum datasets
rgbd_tum_path = ...
    '/Users/MatthewWilmes/Documents/MATLAB/School/EECS568_MATLAB/Project/rgbd_tum/';
% Select the dataset you want to use
dataset_name = 'freiburg1_xyz';

% Set the first row and last row you want to read from the assoc.txt file
start_row = 1; % set to 1 if you want to start at the beginning of the file
end_row = inf; % set to inf if you want to go to the end of the file

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Generate name of folder path to full rgb images
% rgb_path = strcat(dataset_path,'rgb');
% % Generate name of folder path to full depth images
% depth_path = strcat(dataset_path,'depth');

% Generate name of folder path to dataset
dataset_path = strcat(rgbd_tum_path, dataset_name, '/');

scale_pointclouds_and_images(fx, fy, cx, cy, depth_scaling_factor, ...
    scale_num, dataset_path, start_row, end_row);

% pcd_edge_path = strcat(dataset_path,scale_folder,'pcd_edge');
% 
% 
% 
% % Make a pcd_edge subfolder inside the dataset folder if it doesn't exist
% % pcd_edge_path = strcat(dataset_path,'pcd_edge');
% if ~exist(pcd_edge_path,'dir')
%     mkdir(pcd_edge_path);
%     addpath(pcd_edge_path);
% end


