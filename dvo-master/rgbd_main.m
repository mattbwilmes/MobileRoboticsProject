%{
This script processes any dataset of rgb and depth images found here:
https://vision.in.tum.de/data/datasets/rgbd-dataset/download

The user must download the dataset as a .tgz file and unzip it in the
proper folder structure:
-->dataset_path
---->dataset_name
------>scale_1
-------->unzipped contents of .tgz file
-------->assoc.txt (provided by user)
------>scale_05
-------->rgb
-------->pcd_edge
------>scale_025
-------->rgb
-------->pcd_edge
------>etc.


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

% Number of times to down-scale the images
% The resulting scale_vec will start with 1 and divide by factor of 2 scale_num times
% e.g. scale_num = 4 --> [1 0.5 0.25 0.125 0.0625]
%scale_num = 4;
scale_num = 2;

depth_scaling_factor = 5000;  % depth scaling factor

% % TODO: Make this a custom path based on user
% Path to the folder containing your rgbd_tum datasets
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
end_row = inf; % set to inf if you want to go to the end of the file

% Set this flag to true if you want to see the full point cloud output
%   instead of the edges. NOTE: This is only useful for frames that have a
%   large offset
view_full_ptcloud = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
total_time = 0;

% Generate name of folder path to dataset
dataset_path = strcat(rgbd_tum_path, dataset_name, '/');
% Generate names of folders to full .pcd files
pcd_full_path = strcat(dataset_path,'scale_1/pcd_full');

% Create vector of down-scale values
scale_vec = zeros(1,scale_num);
for power = 1:scale_num
    scale_vec(power) = 1/(2^power);
end
scale_vec = [1 scale_vec];

% TODO: Hard-coded for now, need to make this adaptable
%scale_folder = {'scale_1/','scale_05/','scale_025/','scale_0125/','scale_00625/'};
scale_folder = {'scale_1/','scale_05/','scale_025/'};

pcd_edge_path = cell(length(scale_folder),1);
% TODO: Hard-coded for now, need to make this adaptable
% Generate names of folders to edge .pcd files
pcd_edge_path{1} = strcat(dataset_path,'scale_1/pcd_edge');
pcd_edge_path{2} = strcat(dataset_path,'scale_05/pcd_edge');
pcd_edge_path{3} = strcat(dataset_path,'scale_025/pcd_edge');
pcd_edge_path{4} = strcat(dataset_path,'scale_0125/pcd_edge');
pcd_edge_path{5} = strcat(dataset_path,'scale_00625/pcd_edge');

% Generate .pcd files from dataset if true
if make_pcd_files
    generate_pointclouds(fx, fy, cx, cy, depth_scaling_factor, ...
        dataset_path, pcd_full_path, pcd_edge_path, start_row, end_row);
    scale_pointclouds_and_images(fx, fy, cx, cy, depth_scaling_factor, ...
        scale_num, dataset_path, start_row, end_row);
end

% Get the directory info of all .pcd files in the pcd_edge_path folder
pcd_edge_dir_info = dir(fullfile(pcd_edge_path{1}, '*.pcd'));
% Make a cell of these filepath names for each scale
num_pcd_files = length(pcd_edge_dir_info);
ptcloud_files = cell(num_pcd_files,2,length(scale_folder));
% Remove non-alphabet letters from image extension in case a period was
%   included
image_suffix(isletter(image_suffix)==0)=[];

% Initialize cell matrix for transformations
transformation = cell(num_pcd_files-1,2);

for iter = 1:num_pcd_files
    % Get name of the iter'th pcd file
    pcd_file = pcd_edge_dir_info(iter).name;
    % Get name of the corresping rgb file
    rgb_file = replace(pcd_file,'pcd',image_suffix);
    for scale_index = 1:length(scale_folder)
        % Add .pcd file to ptcloud_files cell
        ptcloud_files{iter,1,scale_index} = ...
            fullfile(pcd_edge_path{scale_index}, pcd_file);
        % Add corresponding rgb file to same row of ptcloud_files cell
        ptcloud_files{iter,2,scale_index} = ...
            fullfile(dataset_path, scale_folder{scale_index}, 'rgb/', rgb_file);
    end
end

% Construct an instance of the rgbd_dvo class
rgbd_dvo = rgbd_dvo();

% Initialize target and source pointclouds
target_ptcloud = [];
source_ptcloud = [];

total_time = total_time + toc;

% Coarse-to-fine approach

%         Load the target point cloud
%         target_ptcloud.ptcloud = pcread(ptcloud_files{index_source_ptcloud-1,1,length(scale_vec)});
%         target_ptcloud.ptcloud = pcread(ptcloud_files{1,1,length(scale_vec)});
%         Load the corresponding image as grayscale
%         target_ptcloud.image = rgb2gray(imread(ptcloud_files{index_source_ptcloud-1,2,length(scale_vec)}));
%         target_ptcloud.image = rgb2gray(imread(ptcloud_files{1,2,length(scale_vec)}));

tic
% Index from the second .pcd to the last .pcd
for index_source_ptcloud = 2

    % Index from the smallest scale to full scale
    for index_scale = length(scale_vec):-1:1

        % Scale the camera parameters in proportion with the camera image
        rgbd_dvo.fx = fx * scale_vec(index_scale);
        rgbd_dvo.fy = fy * scale_vec(index_scale);
        rgbd_dvo.cx = cx * scale_vec(index_scale);
        rgbd_dvo.cy = cy * scale_vec(index_scale);
      
        % Load the target point cloud
        target_ptcloud.ptcloud = ...
            pcread(ptcloud_files{index_source_ptcloud-1,1,index_scale});
        target_ptcloud_filtered.ptcloud = ...
            pcRangeFilter(target_ptcloud.ptcloud, ...
            0.9*max(target_ptcloud.ptcloud.Location(:,3)), 0);
        % Load the corresponding image as grayscale
%         target_ptcloud.image = ...
%             rgb2gray(imread(ptcloud_files{index_source_ptcloud-1,2,index_scale}));
        target_ptcloud_filtered.image = ...
            rgb2gray(imread(ptcloud_files{index_source_ptcloud-1,2,index_scale}));
        
        % Load the source point cloud
        source_ptcloud.ptcloud = ...
            pcread(ptcloud_files{index_source_ptcloud,1,index_scale});
        source_ptcloud_filtered.ptcloud = ...
            pcRangeFilter(source_ptcloud.ptcloud, ...
            0.9*max(target_ptcloud.ptcloud.Location(:,3)), 0);
        % Load the corresponding rgb image as grayscale
%         source_ptcloud.image = ...
%             rgb2gray(imread(ptcloud_files{index_source_ptcloud,2,index_scale}));
        source_ptcloud_filtered.image = ...
            rgb2gray(imread(ptcloud_files{index_source_ptcloud,2,index_scale}));


        % Set the point clouds
%         rgbd_dvo.set_ptclouds( ...
%             target_ptcloud, source_ptcloud.ptcloud);
        rgbd_dvo.set_ptclouds( ...
            target_ptcloud_filtered, source_ptcloud_filtered.ptcloud);
        
        % Align the point clouds
        rgbd_dvo.align();
        rgbd_dvo.tform.T

    end

    % Plot full point clouds and transformed point clouds if true
    if view_full_ptcloud
        % Generate filenames for the full point clouds
        target_ptcloud_filename = ...
            replace(ptcloud_files{1,1,1},'pcd_edge','pcd_full');
        source_ptcloud_filename = ...
            replace(ptcloud_files{2,1,1},'pcd_edge','pcd_full');

        % Read full target and source point clouds
        target_ptcloud_full = pcread(target_ptcloud_filename);
        source_ptcloud_full = pcread(source_ptcloud_filename);
        % Apply final transform to full source point cloud
        source_ptcloud_full_transformed = pctransform(source_ptcloud_full, rgbd_dvo.tform);

        % Plot original point clouds on top of one another to show misalignment
        figure(2*index_source_ptcloud-1)
        pcshow(target_ptcloud_full)
        hold on
        pcshow(source_ptcloud_full)
        view(0,-90)
        title('Target and Source Point Clouds without Transform')

        % Plot point clouds on top of one another to show improved alignment
        figure(2*index_source_ptcloud)
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
        figure(2*index_source_ptcloud-1)
        pcshow(target_ptcloud_edge)
        hold on
        pcshow(source_ptcloud_edge)
        view(0,-90)
        title('Target and Source Point Clouds without Transform')
    
        % Plot point clouds on top of one another to show improved alignment
        figure(2*index_source_ptcloud)
        pcshow(target_ptcloud_edge)
        hold on
        pcshow(source_ptcloud_edge_transformed)
        view(0,-90)
        title('Target and Source Point Clouds with Transform')
    end

    rgbd_dvo.tform.T
    sum(rgbd_dvo.residual'*rgbd_dvo.residual)
    
    % Set up for the next iteration, unless there is no next iteration
    if index_source_ptcloud ~= num_pcd_files
        % Choose the next target point cloud
        % % The source point cloud is now the target point cloud for the next pair
        target_ptcloud.ptcloud = source_ptcloud.ptcloud;
        % % Assocaite the proper grayscale image
        target_ptcloud.image = source_ptcloud.image;
        % Load the next source point cloud
        source_ptcloud.ptcloud = pcread(ptcloud_files{index_source_ptcloud+1,1});
        % % Load the corresponding rgb image as grayscale
        source_ptcloud.image = rgb2gray(imread(ptcloud_files{index_source_ptcloud+1,2}));
    end

    % Reset the predicted transformation matrix
    rgbd_dvo.R = eye(3);
    rgbd_dvo.T = zeros(3,1);
    rgbd_dvo.R_prev = eye(3);
    rgbd_dvo.T_prev = zeros(3,1);
    % Reset number of iterations
    rgdb_dvo.iterations = 0;

    % The timestamp is the filename of the source point cloud (minus the extension)
    timestamp = str2double(pcd_edge_dir_info(index_source_ptcloud-1).name(1:end-4));
    % Add timestamp to transformation cell
    transformation{index_source_ptcloud-1,1} = timestamp;
    % Add final transformation to corresponding cell
    transformation{index_source_ptcloud-1,2} = rgbd_dvo.tform.T';

    fprintf('Just finished transform from %d to %d\n',index_source_ptcloud,index_source_ptcloud-1)
    total_time = total_time + toc

end

% Save transformation cell as a .mat file
mat_file = strcat(dataset_path,dataset_name,'_tform','.mat');
save(mat_file,'transformation');
disp('done!')